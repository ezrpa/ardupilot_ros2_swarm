# TAK client certificates (not committed)

Mounted read-only into `swarm_controller` at `/root/tak`. Only needed for
`tls://` TAK Server connections — multicast (`udp://239.2.3.1:6969`) and plain
`tcp://...:8087` need no certs.

Convert the `.p12` from your TAK Server data package / admin:

```bash
openssl pkcs12 -in swarm.p12 -clcerts -nokeys -out tak/swarm.pem
openssl pkcs12 -in swarm.p12 -nocerts -nodes  -out tak/swarm.key
openssl pkcs12 -in truststore-root.p12 -nokeys -out tak/ca.pem
```

Then `TAK_CERT=/root/tak/swarm.pem TAK_KEY=/root/tak/swarm.key
TAK_CA=/root/tak/ca.pem`. See `docs/TAK.md`.
