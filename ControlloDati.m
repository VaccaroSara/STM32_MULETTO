clear
clc

s = serialport("COM5",115200);

flush(s);

N = 500;

ref = zeros(1,N);
meas = zeros(1,N);

figure

for k = 1:N

    data = readline(s);

    values = sscanf(data,'%f,%f');

    if length(values) == 2

        ref(k)  = values(1);
        meas(k) = values(2);

        plot(ref,'--')
        hold on
        plot(meas,'LineWidth',2)
        hold off

        legend('Reference','Measured')
        xlabel('Samples')
        ylabel('RPM')

        grid on

        drawnow
    end
end