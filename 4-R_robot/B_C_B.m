% Timing law --- Bang-coast-bang
% Ascissa curvilinea
i = 1;
s = [];
if L >(V_max^2/a_max)
    T_s = V_max/a_max;
    T_tot = ((L*a_max+V_max^2)/(a_max*V_max));
    dt = (T_tot/interv);
    while t < T_s
        s = [s, 0.5*a_max*t^2];
        t = (t+dt);
        i = i+1;
    end
    while (t > T_s) && (t < (T_tot - T_s))
        s = [s, 0.5*a_max*T_s^2 + V_max*(t-T_s)];
        t = (t+dt);
        i = i+1;
    end
    while (t > (T_tot - T_s)) && (t < T_tot)
        s = [s, 0.5*a_max*T_s^2 + V_max*(T_tot-2*T_s) + 0.5*a_max*(t-T_tot+T_s)^2];
        t = (t+dt);
        i = i+1;
    end
else
    T_tot = sqrt(L/(2*a_max));
    dt = T_tot/interv;
    while t < T_tot/2
        s = [s, 0.5*a_max*t^2];
        t = t+dt;
        i=i+1
    end
    while (t > T_tot/2) &&  (t < T_tot)
        s = [s, 0.5*a_max*(T_max/2)^2+0.5*a_max*(t-T_tot/2)^2];
        t = t+dt;
        i=i+1
    end
end

%Velocità
i = 1;
s_dot = [];
t=0;
if L >(V_max^2/a_max)
    T_s = V_max/a_max;
    T_tot = ((L*a_max+V_max^2)/(a_max*V_max));
    dt = (T_tot/interv);
    while t <= T_s
        s_dot = [s_dot, a_max*t];
        t = (t+dt);
        i = i+1;
    end
    while (t > T_s) && (t <= (T_tot - T_s))
        s_dot = [s_dot, V_max];
        t = (t+dt);
        i = i+1;
    end
    while (t > (T_tot - T_s)) && (t <= T_tot)
        s_dot = [s_dot, V_max - a_max*(t-T_tot+T_s)];
        t = (t+dt);
        i = i+1;
    end
    s_dot(i) = 0;
else
    T_tot = sqrt(L/(2*a_max));
    dt = T_tot/interv;
    while t <= T_tot/2
        s = [s, a_max*t];
        t = t+dt;
        i = i+1;
    end
    while (t > T_tot/2) &&  (t <= T_tot)
        s = [s, a_max*T_tot/2-a_max*(t-T_tot/2)];
        t = t+dt;
        i = i+1;
    end
    s_dot(i) = 0
end
