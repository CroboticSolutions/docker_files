# Sourcea ga entrypoint (nakon kloniranja), ~/.bashrc i save_calibration.sh.
# Postav = DEMOMOTION_CALIBRATION_SET iz .env, a ako ga nema, ime računala
# (DEMOMOTION_HOSTNAME, postavi ga demomotion.sh up). Kalibracije se čitaju i pišu
# ravno u /root/calibration/<postav> (checkout demomotion_calibration); prazna mapa
# = nominalne CAD vrijednosti dok se ne kalibrira. Generička imena računala se ne
# koriste jer bi se dva računala mogla sudariti; tada ostaje ~/.ros/calibration.
_dm_host="${DEMOMOTION_HOSTNAME:-$(hostname)}"
if [ -z "${DEMOMOTION_CALIBRATION_SET:-}" ]; then
  case "$_dm_host" in
    ""|localhost|ubuntu|debian|docker-desktop) ;;
    *) DEMOMOTION_CALIBRATION_SET="$_dm_host" ;;
  esac
fi
if [ -n "${DEMOMOTION_CALIBRATION_SET:-}" ]; then
  export DEMOMOTION_CALIBRATION_SET
  export ARMS_CALIBRATION_DIR="/root/calibration/${DEMOMOTION_CALIBRATION_SET}"
  mkdir -p "$ARMS_CALIBRATION_DIR"
fi
unset _dm_host
