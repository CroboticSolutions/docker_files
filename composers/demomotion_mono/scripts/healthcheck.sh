#!/usr/bin/env bash
# Zdrav = runtime service (vlasnik ROS procesa) i gateway s GUI-jem odgovaraju.
# Ne provjerava robot ni kamere: to GUI prikazuje kroz svoje readiness gateove.
curl -fsS --max-time 3 http://127.0.0.1:8092/health >/dev/null &&
  curl -fsS --max-time 3 -o /dev/null http://127.0.0.1:8080/
