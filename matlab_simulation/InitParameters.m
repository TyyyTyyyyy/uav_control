%%
%�����嵥
clear
path('./icon/',path);

%������Ԫ����
PowerUnit_CR = 1148;%������������ʾ�����ŵ����ת�ٵ����Թ�ϵб��
PowerUnit_wb = -141.4;%ת�ٳ�����Ϊ�����ŵ����ת�����Թ�ϵ�еĳ�����
PowerUnit_Tm = 0.02;%�����̬��Ӧʱ�䳣��

%����Ч�ʵ�Ԫ����
ConEfficiency_drone_R = 0.225;%����뾶����λ��
ConEfficiency_drone_cT = 1.105e-05;%����������ϵ��
ConEfficiency_drone_cM = 1.779e-07;%����������ϵ��

%λ�ö���ѧ����
PosDyna_Mass = 1.4;%����������������λKg
PosDyna_GravityAcc = 9.8;%�������ٶ�

%��̬����ѧ
AttDyna_Ixx = 0.0211;%����x��ת������
AttDyna_Iyy = 0.0219;%����y��ת������
AttDyna_Izz = 0.0366;%����z��ת������
AttDyna_JRP = 0.0001287;%���+��������ת������
