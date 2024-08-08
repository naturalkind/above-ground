sudo systemctl stop aim.service
sudo systemctl daemon-reload
sudo systemctl enable aim.service
sudo systemctl start aim.service
sleep 3
sudo systemctl status aim.service
