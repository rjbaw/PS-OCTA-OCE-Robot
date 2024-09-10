netsh interface portproxy reset
Set-NetConnectionProfile -InterfaceAlias "Ethernet 3" -NetworkCategory Private
netsh interface portproxy add v4tov4 listenport=50001 listenaddress=0.0.0.0 connectport=50001 connectaddress=$(wsl hostname -I)
netsh interface portproxy add v4tov4 listenport=50002 listenaddress=0.0.0.0 connectport=50002 connectaddress=$(wsl hostname -I)
netsh interface portproxy add v4tov4 listenport=50003 listenaddress=0.0.0.0 connectport=50003 connectaddress=$(wsl hostname -I)
netsh interface portproxy add v4tov4 listenport=50004 listenaddress=0.0.0.0 connectport=50004 connectaddress=$(wsl hostname -I)
netsh interface portproxy add v4tov4 listenport=50005 listenaddress=0.0.0.0 connectport=50005 connectaddress=$(wsl hostname -I)

netsh interface portproxy add v4tov4 listenport=50001 listenaddress=0.0.0.0 connectport=50001 connectaddress=127.0.0.1
netsh interface portproxy add v4tov4 listenport=50002 listenaddress=0.0.0.0 connectport=50002 connectaddress=127.0.0.1
netsh interface portproxy add v4tov4 listenport=50003 listenaddress=0.0.0.0 connectport=50003 connectaddress=127.0.0.1
netsh interface portproxy add v4tov4 listenport=50004 listenaddress=0.0.0.0 connectport=50004 connectaddress=127.0.0.1
netsh interface portproxy add v4tov4 listenport=50005 listenaddress=0.0.0.0 connectport=50005 connectaddress=127.0.0.1

netsh interface portproxy add v4tov4 listenport=50001 listenaddress=0.0.0.0 connectport=50001 connectaddress=0.0.0.0
netsh interface portproxy add v4tov4 listenport=50002 listenaddress=0.0.0.0 connectport=50002 connectaddress=0.0.0.0
netsh interface portproxy add v4tov4 listenport=50003 listenaddress=0.0.0.0 connectport=50003 connectaddress=0.0.0.0
netsh interface portproxy add v4tov4 listenport=50004 listenaddress=0.0.0.0 connectport=50004 connectaddress=0.0.0.0
netsh interface portproxy add v4tov4 listenport=50005 listenaddress=0.0.0.0 connectport=50005 connectaddress=0.0.0.0
