function usat = saturate(u,umax)
for ii = 1:length(u)
    if u(ii)>umax
        u(ii) = umax;
    elseif u(ii)<-umax
        u(ii) = -umax;
    end
end
usat = u;
    
        