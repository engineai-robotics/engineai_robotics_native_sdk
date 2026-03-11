#!/usr/bin/env bash
sudo touch /var/run/tail.pid
sudo tail -f /var/run/tail.pid
