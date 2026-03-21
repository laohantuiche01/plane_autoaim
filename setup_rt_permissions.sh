#!/bin/bash

# Ensure the script is run as root
if [ "$EUID" -ne 0 ]; then
  echo "Please run as root: sudo bash $0"
  exit 1
fi

# Get the user who invoked sudo, or fallback to current user
TARGET_USER=${SUDO_USER:-$(whoami)}

# Check if the configuration already exists to avoid duplicates
if grep -q "^$TARGET_USER.*nice.*-20" /etc/security/limits.conf; then
  echo "Nice limits are already configured for user: $TARGET_USER"
else
  echo "$TARGET_USER soft nice -20" >> /etc/security/limits.conf
  echo "$TARGET_USER hard nice -20" >> /etc/security/limits.conf
  echo "Successfully added nice=-20 limits for user $TARGET_USER to /etc/security/limits.conf"
fi

echo "====================================================================="
echo "Configuration complete! "
echo "IMPORTANT: You must LOG OUT and LOG BACK IN (or reboot) for these changes to take effect."
echo "====================================================================="
