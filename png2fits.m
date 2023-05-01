clear;
clc;

X = imread('ariya.png');
R = X(:,:,1);
G = X(:,:,2);
B = X(:,:,3);

fitswrite(R,'ariya.fits','Compression','rice');
fitswrite(G,'ariya.fits','WriteMode','append','Compression','rice');
fitswrite(B,'ariya.fits','WriteMode','append','Compression','rice');

fitsdisp('ariya.fits');
info = fitsinfo('ariya.fits');
img = fitsread('ariya.fits','image');

newimg = zeros(371,1140,3);
newimg(:,:,1)=img(:,:);
newimg(:,:,2)=img(:,:);
newimg(:,:,3)=img(:,:);
newimg = newimg*0.004;
imwrite(newimg,'body.png');