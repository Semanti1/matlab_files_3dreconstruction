%temple 0114,0115
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
tform=[1.0293, -0.0442, -0.0107;  0.0387,  0.9974, -0.0047;0,0,1];
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
% translationOfCamera2 = [10, 0, 0];
% R = eye(3);% rotationOfCamera2, align with the wolrd coordinate.
% T1 = [0 0 0];
% T2 = translationOfCamera2;% translationOfCamera2.
% extrinsicMtx1 = [R,T1'];
% extrinsicMtx2 = [R,T2'];
% 
% opticalCenter = [500 600];
% fx = 4; fy = 4;
% sx = 1/14; sy = 1/14;
% cx = 2000; cy = 1500;
% intrinsicMtx = [fx/sx, 0, 0; 0, fy/sy, 0; cx, cy, 1];
% cam1=intrinsicMtx*extrinsicMtx1;
% cam2=intrinsicMtx*extrinsicMtx2;
k1=[1520.400000 0.000000 302.320000; 0.000000 1525.900000 246.870000; 0.000000 0.000000 1.000000];
Rt1 = [0.52923228321742100000 0.84676991697276738000 0.05379496361564442600;...
    -0.16580700833641160000 0.04103360273833275800 0.98530415579801622000;...
    0.83211851703168138000 -0.53037435001727395000 0.16211669394837064000];
t1 = [-0.04544525507309407600 -0.01091943008098561300 0.60984142685827880000];
    
cam1 = k1*[Rt1,t1'];
k2 =[1520.400000 0.000000 302.320000; 0.000000 1525.900000 246.870000; 0.000000 0.000000 1.000000];
Rt2 = [0.51001885917468270000 0.84999090415045009000 0.13189475405661866000;...
    -0.31438736995305322000 0.04147816357661442800 0.94838818189616503000;...
    0.80065057603186485000 -0.52516190342389890000 0.28838105050788343000];
t2 = [-0.04586621709534437200 -0.01229413226310582100 0.60915680788808313000];
cam2 = k2*[Rt2,t2'];
world = triangulate(x',y',cam1',cam2');
scaled = world.*10000;
figure
plot(scaled(:,1),scaled(:,2))
ptCld = pointCloud(scaled);
figure
pcshow(ptCld)
title('ptcld')