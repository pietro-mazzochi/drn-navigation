#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <csignal>
#include <sys/types.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <filesystem>
#include <string>
#include <vector>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <std_srvs/srv/empty.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <nav2_msgs/srv/save_map.hpp>
#include <filesystem>
#include <std_msgs/msg/int32.hpp>
#include <sys/types.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <unistd.h>
#include <csignal>
#include <memory>
#include <string>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <thread>
#include <fstream>  
#include <sstream>  
#include <system_error> 

using namespace std::chrono_literals;
using std::placeholders::_1;

void handle_sigchld([[maybe_unused]] int sig)
{
    while (waitpid(-1, nullptr, WNOHANG) > 0)
    {
    }
}

class ProcessManager
{
public:
    static pid_t startProcess(const std::vector<std::string> &command, const std::string &log_file, rclcpp::Logger logger)
    {
        pid_t pid = fork();
        if (pid == 0)
        {
            int fd = open(log_file.c_str(), O_RDWR | O_CREAT | O_APPEND, 0600);
            if (fd == -1)
            {
                RCLCPP_ERROR(logger, "Failed to open log file: %s", log_file.c_str());
                _exit(EXIT_FAILURE);
            }
            dup2(fd, STDOUT_FILENO);
            dup2(fd, STDERR_FILENO);
            close(fd);

            std::vector<const char *> args;
            for (const auto &arg : command)
                args.push_back(arg.c_str());
            args.push_back(nullptr);

            execvp(args[0], const_cast<char *const *>(args.data()));
            RCLCPP_ERROR(logger, "Failed to execute process: %s", command[0].c_str());
            _exit(EXIT_FAILURE);
        }
        else if (pid < 0)
        {
            RCLCPP_ERROR(logger, "Failed to fork process for command: %s", command[0].c_str());
        }
        return pid;
    }

    static void terminateProcess(const std::string &process_name, rclcpp::Logger logger)
    {
        //std::string cmd = "killall -SIGTERM -x " + process_name;
        std::string cmd = "killall -e -s SIGTERM " + process_name;

        if (system(cmd.c_str()) != 0)
        {
            RCLCPP_WARN(logger, "Failed to terminate process %s using killall.", process_name.c_str());
        }
        else
        {
            RCLCPP_INFO(logger, "Successfully terminated process: %s", process_name.c_str());
        }
    }

    static bool isProcessRunning(const std::string &name)
    {
        std::string cmd = "pgrep -x " + name + " > /dev/null";
        return (system(cmd.c_str()) == 0);
    }
};

class SlamSubscriber : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    pid_t process_pid_;

public:
    SlamSubscriber()
        : Node("slam_controller"), process_pid_(-1)
    {
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "/switch_nav_mode", 10, std::bind(&SlamSubscriber::toggleCallback, this, _1));
        RCLCPP_INFO(this->get_logger(), "SlamSubscriber node started.");
    }

    void toggleCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received message Int32: %d", msg->data);

        if (msg->data == 1)
        {            
            RCLCPP_INFO(this->get_logger(), "Starting SLAM...");
            //deactivateNavigation();            
            terminate(2);
            terminate(1);
            activateSlamToolbox();            
        }
        else if (msg->data == 3)
        {
            RCLCPP_INFO(this->get_logger(), "Starting navegation...");

            const std::string package_share_directory = ament_index_cpp::get_package_share_directory("drn_navigation_mapping");
            const std::string map_yaml_path = std::filesystem::path(package_share_directory) / "maps" / "map" / "map.yaml";

            if (!std::filesystem::exists(map_yaml_path))
            {
                RCLCPP_ERROR(this->get_logger(), "Map file not found. Aborting localization startup.");
                return;
            }

            terminate(3);
            //activateNavigation(); //teste
        }
        else if ((msg->data == 0 || msg->data == 2))
        {
            if (msg->data == 0)
            {
                if (!saveMap())
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to save map.");
                }
            }
            terminate(3);
            //activateNavigation(); //Teste
            RCLCPP_INFO(this->get_logger(), "All processes terminated.");
        }
        else if (msg->data == 4)
        {
            terminate(3);
            //deactivateNavigation();
        }
    }

    template <typename ServiceT>
    bool waitForServiceAvailability(
        typename rclcpp::Client<ServiceT>::SharedPtr client,
        const std::string &service_name,
        int timeout_sec = 5)
    {
        auto start = std::chrono::steady_clock::now();
        while (rclcpp::ok() && !client->wait_for_service(std::chrono::milliseconds(200)))
        {
            RCLCPP_INFO(rclcpp::get_logger("lifecycle_manager_client"), "Waiting for service: %s", service_name.c_str());
            auto elapsed = std::chrono::steady_clock::now() - start;
            if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() >= timeout_sec)
            {
                RCLCPP_ERROR(rclcpp::get_logger("lifecycle_manager_client"), "Service %s not available after %d seconds", service_name.c_str(), timeout_sec);
                return false;
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("lifecycle_manager_client"), "Service %s is now available", service_name.c_str());
        return true;
    }

    void activateNavigation()
    {
        startProcess({"ros2", "launch", "drn_integration_pointcloud_to_laserscan", "pointcloud2_to_laserscan_launch.py"}, "/tmp/pointcloud2_to_laserscan_output.log");

        //std::vector<std::string> nodes = {
        //    "/controller_server", "/planner_server", "/recoveries_server", "/bt_navigator", 
        //    "/filter_mask_server", "/costmap_filter_info_server", "/speeds_filter_mask_server", "/speeds_costmap_filter_info"};

        call_script("drn_navigation_mapping", "start_localization.bash"); //TODO analisar teste

        call_script("drn_navigation_mapping", "start_navigation.bash"); //TODO analisar teste
    }

    void activateSlamToolbox()
    {
        startProcess({"ros2", "launch", "drn_integration_pointcloud_to_laserscan", "pointcloud_to_laserscan_launch.py"}, "/tmp/pointcloud_to_laserscan_output.log");
        startProcess({"ros2", "launch", "drn_navigation_mapping", "drn_navigation_slam_toolbox_launch.py"}, "/tmp/slam_toolbox_output.log");
    }

    void deactivateNavigation()
    {
        std::vector<std::string> nodes = {
            "/controller_server", "/planner_server", "/recoveries_server", "/bt_navigator",
            "/filter_mask_server", "/costmap_filter_info_server", "/speeds_filter_mask_server", "/speeds_costmap_filter_info"};

        for (const auto &n : nodes)
        {
            sendLifecycleManagerCommand(n, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        }
    }

    void sendLifecycleManagerCommand(const std::string &node_name, uint8_t transition)
    {
        auto client = this->create_client<lifecycle_msgs::srv::ChangeState>(node_name + "/change_state");

        if (!waitForServiceAvailability<lifecycle_msgs::srv::ChangeState>(client, node_name + "/change_state", 5)) //5
        {
            RCLCPP_ERROR(this->get_logger(), "Service %s/change_state not available", node_name.c_str());
            return;
        }

        auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        req->transition.id = transition;

        auto future = client->async_send_request(req);

        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready && future.get()->success)
        {
            RCLCPP_INFO(this->get_logger(), "Node %s transitioned to %d successfully.", node_name.c_str(), transition);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Node %s transition to %d failed.", node_name.c_str(), transition);
        }
    }

    void startProcess(const std::vector<std::string> &command, const std::string &log_file)
    {
        process_pid_ = ProcessManager::startProcess(command, log_file, this->get_logger());
        if (process_pid_ > 0)
        {
            RCLCPP_INFO(this->get_logger(), "Process started with PID: %d", process_pid_);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to start process.");
        }
    }

    void terminate(int select)
    {
        if (select == 1)
        {
            std::vector<std::string> processes = {
                "async_slam_toolbox_node", "map_saver_cli", "dummy_pointcloud", "map_server",
                "controller_server", "planner_server", "behavior_server", "bt_navigator",
                "lifecycle_manager", "pointcloud_to_laserscan_node", "pointcloud2_to_laserscan_node"};

            for (const auto &proc : processes)
            {
                ProcessManager::terminateProcess(proc, this->get_logger());
            }

            process_pid_ = -1;
        }
        else if(select == 2)    
        {
            std::vector<std::string> processes = {"async_slam_toolbox_node", "pointcloud2_to_laserscan_node", "pointcloud_to_l"}; 

            for (const auto &proc : processes)
            {
                ProcessManager::terminateProcess(proc, this->get_logger());
            }                 

            call_script("drn_navigation_mapping", "stop_navigation2.bash"); //TODO analisar teste    
            //call_script("drn_navigation_mapping", "stop_localization.bash"); //TODO analisar teste           

            process_pid_ = -1;  
        }
        else if(select == 3)
        {
            std::vector<std::string> processes = {"async_slam_toolbox_node", "pointcloud_to_laserscan_node", "pointcloud_to_l"};

            for (const auto &proc : processes)
            {
                ProcessManager::terminateProcess(proc, this->get_logger());
            }

            process_pid_ = -1;
        }
    }

    bool saveMap()
    {
        // Caminhos originais (sem alterar o local)
        const std::string pkg_share = ament_index_cpp::get_package_share_directory("drn_navigation_mapping");
        const std::filesystem::path map_dir  = std::filesystem::path(pkg_share) / "maps" / "map";
        const std::filesystem::path map_yaml = map_dir / "map.yaml";
        const std::filesystem::path map_pgm  = map_dir / "map.pgm";
        const std::string log_path = "/tmp/map_saver_output.log";

        // 1) Garante que o diretório existe
        std::error_code ec;
        std::filesystem::create_directories(map_dir, ec);
        if (ec) {
            RCLCPP_ERROR(this->get_logger(),
                        "Não foi possível criar %s: %s",
                        map_dir.string().c_str(), ec.message().c_str());
            return false;
        }

        // 2) Remove arquivos antigos (se existirem)
        std::filesystem::remove(map_yaml, ec);
        ec.clear();
        std::filesystem::remove(map_pgm, ec);
        ec.clear();

        // 3) Prefixo do arquivo (-f espera o prefixo, sem extensão)
        const std::string map_prefix = (map_dir / "map").string();

        // 4) Tenta salvar com alguns retries para contornar race do /map
        constexpr int kMaxAttempts = 3;
        for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
            RCLCPP_INFO(this->get_logger(), "Salvando mapa (tentativa %d/%d)...",
                        attempt, kMaxAttempts);

            pid_t save_pid = ProcessManager::startProcess(
                {
                    "ros2", "run", "nav2_map_server", "map_saver_cli",
                    "-f", map_prefix,
                    "--ros-args",
                    // QoS latched para pegar o último /map
                    "-p", "map_subscribe_transient_local:=true",
                    // *** CORREÇÃO: double (segundos), não inteiro ***
                    "-p", "save_map_timeout:=10.0",
                    // Opcional: elimina warning do formato
                    "-p", "image_format:=pgm"
                    // Se o tópico de mapa não for /map, descomente e ajuste:
                    // "-p", "map_topic:=/seu_topico_de_mapa"
                },
                log_path,
                this->get_logger()
            );

            if (save_pid <= 0) {
                RCLCPP_ERROR(this->get_logger(), "Falha ao iniciar map_saver_cli.");
                return false;
            }

            int status = 0;
            if (waitpid(save_pid, &status, 0) == save_pid && WIFEXITED(status) && WEXITSTATUS(status) == 0) {
                RCLCPP_INFO(this->get_logger(), "Mapa salvo com sucesso em: %s", map_dir.string().c_str());
                return true;
            }

            // 5) Exibe saída do map_saver_cli para diagnosticar
            std::string out;
            {
                std::ifstream ifs(log_path);
                if (ifs) {
                    std::stringstream buf; buf << ifs.rdbuf();
                    out = buf.str();
                    RCLCPP_ERROR(this->get_logger(),
                                "Falha ao salvar mapa (tentativa %d). Saída do map_saver_cli:\n%s",
                                attempt, out.c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(),
                                "Falha ao salvar mapa (tentativa %d). Não consegui ler o log em %s",
                                attempt, log_path.c_str());
                }
            }

            // 6) Se foi a corrida de inscrição no /map, espera e tenta de novo
            if (attempt < kMaxAttempts &&
                out.find("Failed to spin map subscription") != std::string::npos)
            {
                RCLCPP_WARN(this->get_logger(), "Map subscription ainda não pronto. Aguardando e tentando novamente...");
                std::this_thread::sleep_for(std::chrono::milliseconds(800));
                continue;
            }

            // Se não é o erro de subscription ou esgotaram as tentativas, aborta.
            break;
        }

        RCLCPP_ERROR(this->get_logger(), "Map saving failed.");
        return false;
    }

    void call_script(std::string directory, std::string file)
    {
        const std::string pkg = ament_index_cpp::get_package_share_directory(directory);
        const std::filesystem::path script = std::filesystem::path(pkg) / "scripts" / file;

        RCLCPP_INFO(this->get_logger(), "script path: %s", script.string().c_str());

        std::error_code ec;
        std::filesystem::permissions(
            script,
            std::filesystem::perms::owner_exec | std::filesystem::perms::owner_read,
            std::filesystem::perm_options::add, ec);

        std::string cmd = "bash \"" + script.string() + "\"";

        int rc = std::system(cmd.c_str());
        if (rc != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Falha ao executar script (%d): %s", rc, cmd.c_str());
        }
    }
};

int main(int argc, char *argv[])
{
    struct sigaction sa;
    sa.sa_handler = &handle_sigchld;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART | SA_NOCLDSTOP;
    if (sigaction(SIGCHLD, &sa, nullptr) == -1)
    {
        perror("sigaction");
        return 1;
    }

    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<SlamSubscriber>();
    executor.add_node(node);

    //node->terminate(true);

    executor.spin();
    rclcpp::shutdown();

    return 0;
}
