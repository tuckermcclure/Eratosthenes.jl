module UDPConnections

export UDPConnection, udp_connect, udp_send, udp_receive, udp_close

struct UDPConnection
    sock::UDPSocket
    target_ip::IPv4
    send_port::Int
    receive_port::Int
end

"""
    UDPConnection

Create an object for communicating over UDP using `udp_send` and `udp_receive`.
Don't forget to close it with `udp_close(conn)` or `close(conn.sock)`!
"""
function UDPConnection(target_ip::IPv4 = ip"127.0.0.1", send_port::Int = 2000, receive_port::Int = 2001)
    return UDPConnection(UDPSocket(), target_ip, send_port, receive_port)
end
# function UDPConnection(sock::UDPSocket = UDPSocket(), target_ip::IPv4 = ip"127.0.0.1", send_port::Int = 2000, receive_port::Int = 2001)
#     return UDPConnection(sock, target_ip, send_port, receive_port)
# end

"""
    udp_connect
"""
function udp_connect(conn::UDPConnection)
    # TCP equivalent:
    # sock = listen(ip"127.0.0.1", 9000)
    # connect(target_ip, send_port)
    # Create a socket and bind it in order to listen. 0.0.0.0 means "any".
    if !bind(conn.sock, ip"0.0.0.0", conn.receive_port)
        warn("Could not bind. Closing socket.")
        close(conn.sock)
    end
end

"""
    udp_connect(conn) do conn ...
"""
function udp_connect(f::Function, conn::UDPConnection)
    try
        udp_connect(conn)
        f(conn)
    finally
        udp_close(conn)
    end
end

"""
    udp_connect(target_ip, send_port, receive_port) do conn ...
"""
function udp_connect(f::Function, target_ip::IPv4, send_port::Int, receive_port::Int)
    conn = UDPConnection(target_ip, send_port, receive_port)
    try
        f(conn)
    finally
        udp_close(conn)
    end
end

"""
    udp_send

Send some data over to the target.

```julia
udp_send(conn, var1, var2, varn)
```
"""
function udp_send(conn::UDPConnection, args...)
    # TCP equivalent
    # write(conn.sock, Ref(args))
    raw = vcat(map(a -> reinterpret(UInt8, isa(a, Array) ? a : [a]), args)...)
    send(conn.sock, conn.target_ip, conn.send_port, raw)
end

"""
    udp_receive

Receive some data from the target. This uses the input arguments as "examples"
to create the mapping to the output arguments.

```julia
var1, var2, varn = udp_receive(conn, var1, var2, varn)
```
"""
function udp_receive(conn::UDPConnection, args...)
    # TCP equivalent
    # return read(conn.sock, Ref(args))[]
    raw = recv(conn.sock) # A bunch of UInt8s will come in.
    outs = [args] # Create an array so we can point to the args tuple.
    n = min(sizeof(args), sizeof(raw)); # Figure out how much to read.
    unsafe_copy!(reinterpret(Ptr{UInt8}, pointer(outs)), pointer(raw), n)
    return outs[1]
end

"""
    udp_close

Close out the UDP socket. Always do this when done. Use try/catch to make sure.
"""
function udp_close(conn::UDPConnection)
    close(conn.sock)
end

end
