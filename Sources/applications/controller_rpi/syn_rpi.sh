REMOTE_USER="sbrobot"
REMOTE_HOST="sbrobot.local"
REMOTE_DIR="/home/sbrobot/sbrobot"

LOCAL_DIR="/home/konrad/ncs/v2.8.0/self-balancing-robot/Sources/applications/controller_rpi/app/controller_rpi"

ssh "$REMOTE_USER@$REMOTE_HOST" "mkdir -p '$REMOTE_DIR'"

scp -r "$LOCAL_DIR/" "$REMOTE_USER@$REMOTE_HOST:$REMOTE_DIR/"

echo "sync to RPi finished!"