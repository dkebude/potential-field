function [F_tot_alpha, F_tot_beta] = potential(axes_hl, alpha, beta, alpha_goal, beta_goal, c_space, plotFlag)
    
    cla(axes_hl, 'reset')

    att_gain = 100;
    rep_gain = 1;

    rho_0 = 10;

    U_att = zeros(length(alpha), length(beta));
    U_rep = zeros(length(alpha), length(beta));
    F_att_alpha_grid = zeros(length(alpha), length(beta));
    F_att_beta_grid = zeros(length(alpha), length(beta));
    F_rep_alpha_grid = zeros(length(alpha), length(beta));
    F_rep_beta_grid = zeros(length(alpha), length(beta));
    for a_i = 1:length(alpha)
        for b_i = 1:length(beta)
            a = alpha(a_i);
            b = beta(b_i);
            U_att(a_i, b_i) = (1/2)*att_gain*((a-alpha_goal).^2+(b-beta_goal).^2);
            F_att_alpha_grid(a_i, b_i) = -att_gain*(a-alpha_goal);
            F_att_beta_grid(a_i, b_i) = -att_gain*(b-beta_goal);
            rho = 15;

            if a_i <= 50
                a_interval = (a_i-(a_i-1)):(a_i+50);
            elseif a_i >= length(alpha)-50
                a_interval = (a_i-50):(a_i+(length(alpha)-a_i-1));
            else
                a_interval = a_i-50:a_i+50;
            end

            if b_i <= 50
                b_interval = (b_i-(b_i-1)):(b_i+50);
            elseif b_i >= length(beta)-50
                b_interval = (b_i-50):(b_i+((length(beta)-b_i)-1));
            else
                b_interval = b_i-50:b_i+50;
            end

            for i = a_interval
                for j = b_interval
                    if(c_space(i,j) == 1)
                        a_obs = alpha(i);
                        b_obs = beta(j);
                        dist = sqrt((a-a_obs+1e-2).^2+(b-b_obs+1e-2)^2);
                    else
                        dist = rho;
                    end
                    if(dist < rho)
                        rho = dist;
                        del_rho_a = (a-a_obs+1e-2)/dist;
                        del_rho_b = (b-b_obs+1e-2)/dist;
                    end
                end
            end

            if rho <= rho_0
                U_rep(a_i, b_i) = (1/2)*rep_gain*(1/rho - 1/rho_0)^2;
                F_rep_alpha_grid(a_i, b_i) = rep_gain*(1/rho - 1/rho_0)*((1/rho)^2)*del_rho_a;
                F_rep_beta_grid(a_i, b_i) = rep_gain*(1/rho - 1/rho_0)*((1/rho)^2)*del_rho_b;
                if(U_rep(a_i, b_i) > 750)
                    U_rep(a_i, b_i) = 750;
                end
            else
                U_rep(a_i, b_i) = 0;
                F_rep_alpha_grid(a_i, b_i) = 0;
                F_rep_beta_grid(a_i, b_i) = 0;
            end
        end
    end

    U_tot = U_att + U_rep;
    F_tot_alpha = F_att_alpha_grid + F_rep_alpha_grid;
    F_tot_beta = F_att_beta_grid + F_rep_beta_grid;

    if(plotFlag == 1)
        axes(axes_hl);
        [alpha_grid, beta_grid] = meshgrid(alpha*180/pi, beta*180/pi);
        meshc(alpha_grid, beta_grid, U_tot);
        drawnow;
    end

end