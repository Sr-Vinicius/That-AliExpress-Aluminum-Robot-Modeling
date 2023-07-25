function [output1] = get_teach_config(r, tcp_client)
%GET_TEACH_CONFIG Summary of this function goes here
%   Detailed explanation goes here
output1 = r.getpos()
output1 = num2str(output1);
write(tcp_client, output1, "string");
end

