% ��������
x = 1:50;
y = x + rand(1,50)*10;

% ���ø�˹ģ���С�ͱ�׼��
r        = 3;
sigma    = 1;
size(y)
y_filted = Gaussianfilter(r, sigma, y);

% ��ͼ�Ա�
figure
plot(x, y, x, y_filted);
title('��˹�˲�');
legend('�˲�ǰ','�˲���','Location','northwest')
