function [Y] = Y_matrix(q1, q2, q3, dq1, dq2, dq3, ddq1, ddq2, ddq3)

Y11 = ddq1;
Y12 = cos(q2)^2*ddq1-dq1*dq2*sin(2*q2);
Y13 = sin(q2)^2*ddq1+dq1*dq2*sin(2*q2);
Y14 = cos(q2+q3)^2*ddq1-dq1*dq2*sin(2*q2+2*q3)-dq1*dq3*sin(2*q2+2*q3);
Y15 = sin(q2+q3)^2*ddq1+dq1*dq2*sin(2*q2+2*q3)+dq1*dq3*sin(2*q2+2*q3);
Y16 = cos(q2+q3)*cos(q2)*ddq1-(dq1*dq3*sin(q3))/2-dq1*dq2*sin(2*q2+q3)-(dq1*dq3*sin(2*q2+q3))/2;
Y17 = 0;
Y18 = 0;
Y19 = sin(q2)*ddq2+dq2^2*cos(q2);
Y110 = sin(q2+q3)*(ddq2+ddq3)+dq2^2*cos(q2+q3)+dq3^2*cos(q2+q3)+2*dq2*dq3*cos(q2+q3);
Y111 = 0;
Y112 = 0;
Y113 = 0;

Y21 = 0;
Y22 = (dq1^2*sin(2*q2))/2;
Y23 = -(dq1^2*sin(2*q2))/2;
Y24 = (dq1^2*sin(2*q2+2*q3))/2;
Y25 = -(dq1^2*sin(2*q2+2*q3))/2;
Y26 = cos(q3)*(2*ddq2+ddq3)-2*dq2*dq3*sin(q3)+(dq1^2*sin(2*q2+q3))/2-dq3^2*sin(q3);
Y27 = ddq2;
Y28 = ddq3;
Y29 = sin(q2)*ddq1;
Y210 = sin(q2+q3)*ddq1;
Y211 = cos(q2);
Y212 = cos(q2+q3);
Y213 = cos(q2);

Y31 = 0;
Y32 = 0;
Y33 = 0;
Y34 = (dq1^2*sin(2*q2+2*q3))/4;
Y35 = -(dq1^2*sin(2*q2+2*q3))/2;
Y36 = cos(q2)*ddq2+dq2^2*sin(q3)+(dq1^2*sin(q3))/4+(dq1^2*sin(2*q2+q3))/4;
Y37 = 0;
Y38 = ddq2+ddq3;
Y39 = 0;
Y310 = sin(q2+q3)*ddq1;
Y311 = 0;
Y312 = cos(q2+q3);
Y313 = 0;

Y = [Y11 Y12 Y13 Y14 Y15 Y16 Y17 Y18 Y19 Y110 Y111 Y112 Y113;
     Y21 Y22 Y23 Y24 Y25 Y26 Y27 Y28 Y29 Y210 Y211 Y212 Y213;
     Y31 Y32 Y33 Y34 Y35 Y36 Y37 Y38 Y39 Y310 Y311 Y312 Y313];
end