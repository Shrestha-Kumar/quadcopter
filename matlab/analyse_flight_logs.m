function log = analyse_flight_logs(csv_file, sim_out)
%ANALYSE_FLIGHT_LOGS  Parse hardware telemetry log from quad_fc and compare with simulation.
%   log = analyse_flight_logs('data/flight1.csv', out_nom_pid)
%   Expects lines produced by `TEL:1` in firmware/quad_fc/quad_fc.ino:
%     T,t_ms,state,roll,pitch,yaw,gx,gy,gz,roll_sp,pitch_sp,yaw_sp,thr,m0,m1,m2,m3

fid = fopen(csv_file, 'r');
if fid < 0, error('Cannot open %s', csv_file); end
raw = [];
while ~feof(fid)
    ln = strtrim(fgetl(fid));
    if startsWith(ln, 'T,')
        parts = str2double(strsplit(ln(3:end), ','));
        if numel(parts) == 16 && all(~isnan(parts))
            raw(end+1,:) = parts; %#ok<AGROW>
        end
    end
end
fclose(fid);
if isempty(raw), error('No valid T,... telemetry lines found in %s', csv_file); end

log.t        = (raw(:,1) - raw(1,1)) * 1e-3;
log.armed    = raw(:,2);
log.eul_deg  = raw(:,3:5);
log.gyro_dps = raw(:,6:8);
log.sp_deg   = raw(:,9:11);
log.thr_us   = raw(:,12);
log.mot_us   = raw(:,13:16);

% Compute tracking RMSE on armed portion
idx = log.armed == 1;
if any(idx)
    err = log.sp_deg(idx,:) - log.eul_deg(idx,:);
    log.rmse_deg = sqrt(mean(err.^2, 1));
    fprintf('Hardware Armed RMSE [Roll, Pitch, Yaw] = [%.2f, %.2f, %.2f] deg\n', log.rmse_deg);
end

figure('Position',[100 100 1000 600]);
names = {'Roll (deg)','Pitch (deg)','Yaw (deg)'};
for k = 1:3
    subplot(2,2,k); hold on; grid on;
    plot(log.t, log.sp_deg(:,k),  'k--', 'LineWidth', 1.2);
    plot(log.t, log.eul_deg(:,k), 'b-',  'LineWidth', 1.5);
    if nargin >= 2 && ~isempty(sim_out)
        plot(sim_out.t, rad2deg(sim_out.eul(:,k)), 'r-.', 'LineWidth', 1.3);
        legend('Setpoint','Hardware','Simulation','Location','best');
    else
        legend('Setpoint','Hardware','Location','best');
    end
    xlabel('Time (s)'); ylabel(names{k}); title(['Hardware vs Simulation: ', names{k}]);
end
subplot(2,2,4); hold on; grid on;
plot(log.t, log.mot_us, 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('PWM (\mus)'); title('Hardware ESC Commands');
legend('M0','M1','M2','M3','Location','best');
end
