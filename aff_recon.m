%dino 1 and 2

img1 = imread('/home/semanti/Documents/cnngeometric_pytorch/source.png');
img2 = imread('/home/semanti/Documents/cnngeometric_pytorch/target.png');
figure
imshow(img1);
title('original img1');
figure
imshow(img2)
title('original img2');
%%
% pin = [1578,1614;1578,1211;2057,1682;2021,1163]';
% pout = [1577,1732;1585,1326;2066,1807;2033,1280]';
% pin =[209,426;572,70;601,174;165,65];
% pout=[206,432;570,68;602,182;167,65];
% tform = estimateGeometricTransform(pin,pout,'projective');
tform=[1.0303,  0.0259,  0.0105;  0.0049,  0.9753, -0.0162; 0,0,1]
img1_bw = imbinarize(rgb2gray(img1),0.3);
img2_bw = imbinarize(rgb2gray(img2),0.3);

xbl = 500;
xbr = 2524;
[x1,y1]=find(img1_bw==1);
[x2,y2]=find(img2_bw==1);
figure
imshow(img1_bw)
hold on;
plot(y1,x1,'rx');
title('points on image1 after segmentation');
figure
imshow(img2_bw);
hold on;
plot(y2,x2,'rx');
title('points on image2 after segmentation');
% aff = tform.T'
% inp=[x1,y1]';
%%
% aff = tform.T'
% aff = tform.T'
aff=tform
inp=[y1,x1]';
pin = inp;
figure
imshow(img1);
hold on;
plot(pin(1,:),pin(2,:),'rx');
title('points in img1')
% hold on;
% plot(1422,1486,'bd');
% x=pin;
x=inp;
q = aff * [x; ones(1, size(x,2))];
% q1 = aff * [1422;1486;1];
p = q(3,:);
y = [q(1,:)./p; q(2,:)./p];
% p1 = q1(3,:);
% y1 = [q1(1,:)./p1; q1(2,:)./p1];
figure
imshow(img2);
hold on;
% plot(pout(1,:),pout(2,:),'rx');
% hold on;
plot(y(1,:),y(2,:),'go');
title('points in img2 mapped using affine')
% hold on;
% plot(y1(1,:),y1(2,:),'bd');
%%
k1=[3310.400000 0.000000 316.730000; 0.000000 3325.500000 200.550000; 0.000000 0.000000 1.000000];
Rt1 = [-0.02755613714744022800 0.99855065785218411000 -0.04622970395378579000 ;...
    -0.97626728342925628000 -0.03682184943054355600 -0.21341815104932660000;...
    -0.21481113274439387000 0.03925223953400933800 0.97586583758523360000 ;];
t1 = [-0.03273720861024858500 -0.00135737867652815920 0.66025659732745012000];
    
cam1 = k1*[Rt1,t1'];
k2 =[3310.400000 0.000000 316.730000; 0.000000 3325.500000 200.550000; 0.000000 0.000000 1.000000];
Rt2 = [-0.02777341537047223900 0.99852890790809612000 -0.04656822454514868800;...
    -0.95382200060751443000 -0.04041118674799367600 -0.29764312460396397000;...
    -0.29908727939432017000 0.03615189149178960100 0.95354003404660426000;];
t2 = [-0.03273968791662332900 -0.00162602232526230790 0.65977471964404810000];
cam2 = k2*[Rt2,t2'];
world = triangulate(x',y',cam1',cam2');
scaled = world.*10000;
figure
plot(scaled(:,1),scaled(:,2))
ptCld = pointCloud(scaled);
figure
pcshow(ptCld)
title('ptcld')