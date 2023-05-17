x1 = 0:0.01:10;
x1 = reshape(x1,size(x1));
y1 = sin(x1);

y1 = awgn(y1,4,'measured');

plot(x1,y1);