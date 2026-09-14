#!/bin/bash
#
# fpga_ping_demo.sh -- show the LitePCIe FPGA network interface carrying real IP traffic.
#
# The FPGA loops the Ethernet stream back on itself (eth_tx -> eth_rx in the top level), so every
# frame the Host sends comes back to it. That alone is NOT enough for ping: the Host would receive
# its own ARP request rather than a reply, ARP would never resolve, and ping would report
# "Destination Host Unreachable".
#
# So this script creates a peer that the Host can only reach *through* the FPGA: a macvlan
# interface in its own network namespace. A Host and its own macvlan normally cannot talk to each
# other, because their frames leave on the physical link and never come back. Here they do come
# back -- through the FPGA. Every ARP and ICMP packet in this demo therefore crosses PCIe into the
# FPGA fabric and back again; nothing is short-circuited in software.
#
# Usage:
#   sudo ./fpga_ping_demo.sh                # set up, ping, show counters, tear down
#   sudo ./fpga_ping_demo.sh --keep         # leave it up afterwards (remove with --down)
#   sudo ./fpga_ping_demo.sh --down         # tear down only
#   sudo ./fpga_ping_demo.sh --iface ethX   # pick the interface (default: enp23s0)
#   sudo ./fpga_ping_demo.sh --vepa         # macvlan VEPA mode, if bridge mode misbehaves
#

set -u

IFACE="enp23s0"
NS="fpgademo"
PEER_IF="peer0"
HOST_IP="10.99.99.1"
PEER_IP="10.99.99.2"
PREFIX="24"
SUBNET="10.99.99.0/24"
PEER_MAC="02:00:00:00:99:02"   # Locally-administered: obviously a test address.
MODE="bridge"
COUNT="5"
KEEP=0
DOWN_ONLY=0

# Colours, only when writing to a terminal.
if [ -t 1 ]; then
    B=$'\e[1m'; G=$'\e[32m'; R=$'\e[31m'; Y=$'\e[33m'; N=$'\e[0m'
else
    B=""; G=""; R=""; Y=""; N=""
fi

say()  { echo "${B}==> $*${N}"; }
ok()   { echo "    ${G}OK${N}   $*"; }
warn() { echo "    ${Y}NOTE${N} $*"; }
err()  { echo "    ${R}FAIL${N} $*" >&2; }

usage() { sed -n '2,23p' "$0" | sed 's/^# \{0,1\}//'; exit 0; }

while [ $# -gt 0 ]; do
    case "$1" in
        --iface) IFACE="${2:-}"; shift 2 ;;
        --count) COUNT="${2:-}"; shift 2 ;;
        --keep)  KEEP=1; shift ;;
        --down)  DOWN_ONLY=1; shift ;;
        --vepa)  MODE="vepa"; shift ;;
        -h|--help) usage ;;
        *) err "Unknown argument: $1"; exit 2 ;;
    esac
done

if [ "$(id -u)" -ne 0 ]; then
    err "Run as root (network configuration)."
    exit 1
fi

# Read one interface counter from sysfs. These come from the driver's ndo_get_stats64, so
# rx_missed_errors is the MAC's hardware RX drop counter.
read_stat() { cat "/sys/class/net/$IFACE/statistics/$1" 2>/dev/null || echo 0; }

teardown() {
    ip netns del "$NS" 2>/dev/null
    # The macvlan goes away with the namespace; only clean up if it was left behind.
    ip link del "$PEER_IF" 2>/dev/null
    ip addr del "$HOST_IP/$PREFIX" dev "$IFACE" 2>/dev/null
    return 0
}

if [ "$DOWN_ONLY" -eq 1 ]; then
    say "Removing the demo setup"
    teardown
    ok "Torn down."
    exit 0
fi

# ---------------------------------------------------------------- checks --

say "Checking the interface"

if [ ! -d "/sys/class/net/$IFACE" ]; then
    err "No interface '$IFACE'. Is the driver loaded? Try: ip link"
    exit 1
fi

DRIVER="$(basename "$(readlink -f "/sys/class/net/$IFACE/device/driver" 2>/dev/null)" 2>/dev/null)"
if [ "$DRIVER" = "litepcie" ]; then
    ok "$IFACE is driven by the litepcie driver."
else
    warn "$IFACE is driven by '${DRIVER:-unknown}', not litepcie. Continuing anyway."
fi
ok "MAC address: $(cat "/sys/class/net/$IFACE/address")"

if ip route show | grep -q "^${SUBNET%/*}"; then
    warn "A route for $SUBNET already exists; the demo may collide with it."
fi

modprobe macvlan 2>/dev/null

# ------------------------------------------------------------------ setup --

# Always start from a clean slate so the script can be re-run safely.
teardown

if [ "$KEEP" -eq 0 ]; then
    trap teardown EXIT
fi

say "Bringing up $IFACE as $HOST_IP/$PREFIX"
ip link set "$IFACE" up                     || { err "Could not bring up $IFACE"; exit 1; }
ip addr add "$HOST_IP/$PREFIX" dev "$IFACE" || { err "Could not set the address"; exit 1; }
ok "Host side ready."

say "Creating a peer reachable only through the FPGA (macvlan, mode $MODE, in netns '$NS')"
ip netns add "$NS"                                               || { err "ip netns add failed"; exit 1; }
ip link add "$PEER_IF" link "$IFACE" type macvlan mode "$MODE"    || { err "macvlan creation failed"; exit 1; }
ip link set "$PEER_IF" address "$PEER_MAC"                       || { err "Could not set the peer MAC"; exit 1; }
ip link set "$PEER_IF" netns "$NS"                               || { err "Could not move the peer into the netns"; exit 1; }
ip netns exec "$NS" ip link set lo up
ip netns exec "$NS" ip link set "$PEER_IF" up                    || { err "Could not bring the peer up"; exit 1; }
ip netns exec "$NS" ip addr add "$PEER_IP/$PREFIX" dev "$PEER_IF" || { err "Could not address the peer"; exit 1; }
ok "Peer $PEER_IP is up on MAC $PEER_MAC."
echo
echo "    The Host and this peer share one physical port and cannot talk directly."
echo "    Every packet between them must go out through the FPGA and come back."
echo

# ------------------------------------------------------------------- ping --

TX0=$(read_stat tx_packets); RX0=$(read_stat rx_packets)

say "Pinging $PEER_IP across the FPGA"
ping -c "$COUNT" -i 0.3 -W 1 "$PEER_IP"
PING_RC=$?
echo

TX1=$(read_stat tx_packets); RX1=$(read_stat rx_packets)
MISSED=$(read_stat rx_missed_errors)

say "What actually happened on the wire"
echo "    Frames transmitted by the FPGA NIC : $((TX1 - TX0))"
echo "    Frames received  by the FPGA NIC   : $((RX1 - RX0))"
echo "    Frames dropped in the MAC (no slot): $MISSED"
echo
echo "    ARP table -- note the peer answered from a different MAC:"
ip neigh show dev "$IFACE" | sed 's/^/      /'
echo

if [ "$PING_RC" -eq 0 ]; then
    ok "Ping succeeded: IP traffic is flowing through the FPGA."
else
    err "Ping failed."
    echo
    echo "    Things to check:"
    echo "      - Is the FPGA still built with the Ethernet loopback? If the User's module has"
    echo "        replaced it, the frames no longer come back and nothing will answer."
    echo "      - Try the other macvlan mode: re-run with --vepa (or without it, for bridge)."
    echo "      - Strict reverse-path filtering can drop the replies:"
    echo "          sysctl -w net.ipv4.conf.$IFACE.rp_filter=2"
    echo "      - Watch the frames directly:"
    echo "          sudo tcpdump -i $IFACE -e -n"
    echo "      - If the counters above did not move, the problem is below IP:"
    echo "        run liteeth_loopback_test.py first."
fi

if [ "$KEEP" -eq 1 ]; then
    echo
    say "Leaving the setup in place (--keep)"
    echo "    Try it yourself:   ping $PEER_IP"
    echo "                       ip -s link show $IFACE"
    echo "    Remove it with:    sudo $0 --iface $IFACE --down"
fi

exit "$PING_RC"
