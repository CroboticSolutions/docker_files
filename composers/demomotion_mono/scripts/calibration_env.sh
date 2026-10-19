# Sourcea ga entrypoint i ~/.bashrc. S DEMOMOTION_CALIBRATION_SET kalibracije se čitaju
# i pišu ravno u checkout demomotion_calibration (/root/calibration/<postav>); bez njega
# ostaje zadani ~/.ros/calibration (prazan = nominalne CAD vrijednosti).
if [ -n "${DEMOMOTION_CALIBRATION_SET:-}" ]; then
  export ARMS_CALIBRATION_DIR="/root/calibration/${DEMOMOTION_CALIBRATION_SET}"
  mkdir -p "$ARMS_CALIBRATION_DIR"
fi
