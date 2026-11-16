function twin_send(q_arm, q_pend, source, u)
% TWIN_SEND  Send one pose to the Unreal twin (twin/RIPTwin).
%   twin_send(q_arm, q_pend)            model pose, angles in rad (q_pend = 0 hanging, pi upright)
%   twin_send(q_arm, q_pend, 'real')    pose of the real rig (what the ESP32 will send)
%   twin_send(..., u)                   reuse an open udpport (see twin_play)
% Packet (16 bytes, little-endian): "RIP1" | uint8 source (0 real, 1 model) | 3 pad | float32 q_arm | float32 q_pend
arguments
    q_arm (1, 1) double
    q_pend (1, 1) double
    source (1, :) char {mustBeMember(source, {'model', 'real'})} = 'model'
    u = []
end
persistent sock
if isempty(u)
    if isempty(sock) || ~isvalid(sock), sock = udpport('datagram', 'IPV4'); end
    u = sock;
end
src = uint8(strcmp(source, 'model'));
pkt = [uint8('RIP1'), src, uint8([0 0 0]), typecast(single([q_arm q_pend]), 'uint8')];
write(u, pkt, 'uint8', '127.0.0.1', 7400);
end
