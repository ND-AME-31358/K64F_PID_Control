function output_data = K64_PID_matlab()
    figure(1);  clf;       % Create an empty figure to update later
    subplot(311)
    h1 = plot([0],[0]);
    h1.XData = []; h1.YData = [];
    ylabel('Position (degs)');
    
    subplot(312)
    h2 = plot([0],[0]);
    h2.XData = []; h2.YData = [];
    ylabel('Velocity (degs/s)');
    
    subplot(313)
    h3 = plot([0],[0]);
    h3.XData = []; h3.YData = [];
    ylabel('Current (A)');
    
    % This function will get called any time there is new data from
    % the FRDM board. Data comes in blocks, rather than one at a time.
    function my_callback(new_data)
        t = new_data(:,1);   % time
        pos = new_data(:,2); % position
        vel = new_data(:,3); % velocity
        current = new_data(:,5); % velocity
        N = length(pos);
        
        h1.XData(end+1:end+N) = t;   % Update subplot 1
        h1.YData(end+1:end+N) = pos;
        h2.XData(end+1:end+N) = t;   % Update subplot 2
        h2.YData(end+1:end+N) = vel;
        h3.XData(end+1:end+N) = t;   % Update subplot 3
        h3.YData(end+1:end+N) = current;
    end
    
    frdm_ip  = '192.168.1.100';     % FRDM board ip
    frdm_port= 11223;               % FRDM board port  
    params.callback = @my_callback; % callback function
    params.timeout  = 2;            % end of experiment timeout
    
    % The example program provided takes two arguments
%     v1 = 1.0;           % pwm applied for first second 
%     v2 = -1.0;           % pwm applied for second second
%     input = [v1 v2];    % input sent to FRDM board
%     output_size = 3;    % number of outputs expected

    angle_des = 90; 
    % Current Controller
    Kp = 0.03;
    Kd = 0.001;
    Ki = 0.0000;
    ExpTime = 3 ; % Expriement time
    
    input = [angle_des Kp Kd Ki ExpTime];
    %input = [1 3.8244 0.0 3 0 0 5];
    output_size = 5;
   
    output_data = RunExperiment(frdm_ip,frdm_port,input,output_size,params);
    
end



