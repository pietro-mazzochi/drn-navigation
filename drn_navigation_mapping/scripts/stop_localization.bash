#!/usr/bin/env bash
set -euo pipefail

PGID_FILE="/tmp/nav_localization.pgid"

if [[ ! -f "$PGID_FILE" ]]; then
  echo "Nenhum PGID encontrado em $PGID_FILE. Já está parado?"
  exit 0
fi

PGID="$(cat "$PGID_FILE" || true)"
if [[ -z "${PGID:-}" ]]; then
  echo "PGID vazio. Removendo marcador."
  rm -f "$PGID_FILE"
  exit 0
fi

echo "Encerrando grupo de processos PGID=$PGID..."
# Tentativa graciosa
kill -TERM -- "-$PGID" 2>/dev/null || true
sleep 3
# Força se ainda restar algo
kill -KILL -- "-$PGID" 2>/dev/null || true

rm -f "$PGID_FILE"
echo "Localização parada e PGID removido."


