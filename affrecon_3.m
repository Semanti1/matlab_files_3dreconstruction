%dino 24,27
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
% tform=[  1.2486,  0.1631, -0.1511; -0.2465,  1.0720,  0.1027;0,0,1]
tform=[1.0215,  0.0141,  0.0038;  0.1574,  0.8682, -0.0070;0,0,1]
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
% for i=1:size(y_orig,2)
%     
%     if (y2(find(x2==y_orig(1))==y_orig(1))
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
Rt1 = [-0.02033168729292493900 0.99844224743179943000 -0.05195800203344297400;...
    0.99444796295706794000 0.02556261082480576100 0.10208229775016617000;...
    0.10325136126036766000 -0.04959470407319268400 -0.99341743300670771000];
t1 = [-0.03272954526114759500 0.00900143999264167200 0.65303441046498523000];
    
cam1 = k1*[Rt1,t1'];
k2 =[3310.400000 0.000000 316.730000; 0.000000 3325.500000 200.550000; 0.000000 0.000000 1.000000];
Rt2 = [-0.01940798101342836000 0.99854198540470829000 -0.05037031639079046800;...
    0.87526353500534249000 0.04132000561940333400 0.48187865649239536000;...
    0.48325774355171169000 -0.03473562610039397600 -0.87478821284621400000];
t2 = [-0.03271835110129187000 0.01013103771622485600 0.65528630010678723000];
cam2 = k2*[Rt2,t2'];
world = triangulate(x',y',cam1',cam2');
scaled = world.*10000;
figure
plot(scaled(:,1),scaled(:,2))
ptCld = pointCloud(scaled);
figure
pcshow(ptCld)
title('ptcld')