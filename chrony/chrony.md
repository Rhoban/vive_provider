# Chrony setup

- Place `chrony_server.conf` on the server under `/etc/chrony/chrony.conf`
  (adapt IPs if needed)

- Place `chrony_robot.conf` on the robot under `/etc/chrony/chrony.conf`
  (adapt IPs if needed)

- On the robots, edit /etc/default/chrony and change `DAEMON_OPTS` to:

	DAEMON_OPTS="-F 0"

