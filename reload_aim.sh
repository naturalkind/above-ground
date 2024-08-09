sudo systemctl daemon-reload
sudo systemctl enable aim.service
sudo systemctl restart aim.service
sleep 3
sudo systemctl status aim.service
