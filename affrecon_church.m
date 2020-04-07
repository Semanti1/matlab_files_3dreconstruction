%church 1,2
close all;
img1 = imread('/home/semanti/Downloads/Church_img/Church1_p1.JPG');
img2 = imread('/home/semanti/Downloads/Church_img/Church1_p2.JPG');
% img1 = imread('/home/semanti/Documents/cnngeometric_pytorch/source.png');
% img2 = imread('/home/semanti/Documents/cnngeometric_pytorch/target.png');
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
% tform=[ 1.0343, -0.0020,  0.0027; -0.0123,  1.0664, -0.1928;0,0,1]%cropped
tform=[1.0343, -0.0020,  0.0027; -0.0123,  1.0664, -0.1928;0,0,1]
img1_bw = imbinarize(rgb2gray(img1),0.3);
img2_bw = imbinarize(rgb2gray(img2),0.3);
% img1_bw=img1_bw(60:180,60:180);
% img2_bw=img2_bw(60:180,60:180);
% img1=img1(60:180,60:180);
% img2=img2(60:180,60:180);
xbl = 500;
xbr = 2524;
[l,w]=size(img1_bw)
[x1,y1]=find(img1_bw==0);
[x2,y2]=find(img2_bw==0);
% indx = find(x1>60 & x1<180);
% indy = find(y1>60 & y1<180);
% indx = find(x1>756 & x1<2268);
% indy = find(y1>756 & y1<2268);
indx = find(x1>300 & x1<2000);
indy = find(y1>300 & y1<3000);
ind = intersect(indx,indy);
x1=x1(ind);
y1=y1(ind);
figure
imshow(img1_bw)
hold on;
plot(y1,x1,'rx');
title('points on image1 after segmentation');
% indx = find(x2>60 & x2<180);
% indy = find(y2>60 & y2<180);
% indx = find(x2>756 & x2<2268);
% indy = find(y2>756 & y2<2268);
indx = find(x2>300 & x2<2000);
indy = find(y2>300 & y2<3000);
ind = intersect(indx,indy);
x2=x2(ind);
y2=y2(ind);
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
% aff= [1 0 10;0 1 0;0 0 1]
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
% aff=[1.0343   -0.0020 -0.1928;-0.0123    1.0664 0.0027;0 0 1];
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
disp('k')
% hold on;
% plot(y1(1,:),y1(2,:),'bd');
%%
    translationOfCamera2 = [10, 0, 0];
% translationOfCamera2 = [0.80,0,0];
% translationOfCamera2 = [2.5, 0, 0];
R = eye(3);% rotationOfCamera2, align with the wolrd coordinate.
T1 = [0 0 0];
T2 = translationOfCamera2;% translationOfCamera2.
extrinsicMtx1 = [R;T1];
extrinsicMtx2 = [R;T2];

opticalCenter = [500 600];
fx = 4; fy = 4;
sx = 1/14; sy = 1/14;
cx = 2000; cy = 1500;
% cx=159;cy=119;
% cx=502;cy=424
intrinsicMtx = [fx/sx, 0, 0; 0, fy/sy, 0; cx, cy, 1];
cam1 = intrinsicMtx'*[R,T1'];
cam2 = intrinsicMtx'*[R,T2'];
% cam1=cam1';
% cam2=cam2';
% k1=[3310.400000 0.000000 316.730000; 0.000000 3325.500000 200.550000; 0.000000 0.000000 1.000000];
% Rt1 = [-0.14396457836077139000 0.96965263281337499000 0.19760617153779569000;...
%     -0.90366580603479685000 -0.04743335255026152200 -0.42560419233334673000;...
%     -0.40331536459778505000 -0.23984130575212276000 0.88306936201487163000];
% t1 = [-0.010415508744 -0.0294278883669 0.673097816109];
%     
% cam1 = k1*[Rt1,t1'];
% k2 =[3310.400000 0.000000 316.730000; 0.000000 3325.500000 200.550000; 0.000000 0.000000 1.000000];
% Rt2 = [-0.23143687262851170000 0.96422332027030622000 0.12926937165558827000;...
%     -0.65860811165749811000 -0.05749426020475298400 -0.75028640486482034000;...
%     -0.71601208187197274000 -0.25878137856376976000 0.64835049619606955000];
% t2 = [-0.0143392676288 -0.0315356332107 0.659314086504];
% cam2 = k2*[Rt2,t2'];
world = triangulate(x',y',cam1',cam2');
scaled = world.*10000;
% scaled = world;
figure
plot(scaled(:,1),scaled(:,2))
ptCld1 = pointCloud(scaled);
figure
pcshow(ptCld1)
title('ptcld1')
axis tight;
ptCld = ptCld1;
figure
tri = delaunay(scaled(:,1),scaled(:,2));
trimesh(triangulation(tri, scaled(:,1), scaled(:,2), scaled(:,3)))
title('mesh')
figure
plot3(scaled(:,1), scaled(:,2), scaled(:,3),'rx')
title('just plot')
% pcwrite(ptCld1,'ptcld1','PLYFormat','binary');
% %%
% 
% %dinoringsparse 2,3
% img1 = imread('/home/semanti/Downloads/dinoSparseRing/dinoSR0002.png');
% img2 = imread('/home/semanti/Downloads/dinoSparseRing/dinoSR0003.png');
% figure
% imshow(img1);
% title('original img1');
% figure
% imshow(img2)
% title('original img2');
% %%
% % pin = [1578,1614;1578,1211;2057,1682;2021,1163]';
% % pout = [1577,1732;1585,1326;2066,1807;2033,1280]';
% % pin =[209,426;572,70;601,174;165,65];
% % pout=[206,432;570,68;602,182;167,65];
% % tform = estimateGeometricTransform(pin,pout,'projective');
% % tform=[  1.2486,  0.1631, -0.1511; -0.2465,  1.0720,  0.1027;0,0,1]
% tform=[0.9899, -0.0722,  0.0091;  0.0051,  1.1334, -0.1003;0,0,1]
% img1_bw = imbinarize(rgb2gray(img1),0.3);
% img2_bw = imbinarize(rgb2gray(img2),0.3);
% 
% xbl = 500;
% xbr = 2524;
% [x1,y1]=find(img1_bw==1);
% [x2,y2]=find(img2_bw==1);
% figure
% imshow(img1_bw)
% hold on;
% plot(y1,x1,'rx');
% title('points on image1 after segmentation');
% figure
% imshow(img2_bw);
% hold on;
% plot(y2,x2,'rx');
% title('points on image2 after segmentation');
% % aff = tform.T'
% % inp=[x1,y1]';
% %%
% % aff = tform.T'
% % aff = tform.T'
% aff=tform
% inp=[y1,x1]';
% pin = inp;
% figure
% imshow(img1);
% hold on;
% plot(pin(1,:),pin(2,:),'rx');
% title('points in img1')
% % hold on;
% % plot(1422,1486,'bd');
% % x=pin;
% x=inp;
% q = aff * [x; ones(1, size(x,2))];
% % q1 = aff * [1422;1486;1];
% p = q(3,:);
% y = [q(1,:)./p; q(2,:)./p];
% % p1 = q1(3,:);
% % y1 = [q1(1,:)./p1; q1(2,:)./p1];
% % for i=1:size(y_orig,2)
% %     
% %     if (y2(find(x2==y_orig(1))==y_orig(1))
% figure
% imshow(img2);
% hold on;
% % plot(pout(1,:),pout(2,:),'rx');
% % hold on;
% plot(y(1,:),y(2,:),'go');
% title('points in img2 mapped using affine')
% % hold on;
% % plot(y1(1,:),y1(2,:),'bd');
% %%
% 
% k1 =[3310.400000 0.000000 316.730000; 0.000000 3325.500000 200.550000; 0.000000 0.000000 1.000000];
% Rt1 = [-0.23143687262851170000 0.96422332027030622000 0.12926937165558827000;...
%     -0.65860811165749811000 -0.05749426020475298400 -0.75028640486482034000;...
%     -0.71601208187197274000 -0.25878137856376976000 0.64835049619606955000];
% t1 = [-0.0143392676288 -0.0315356332107 0.659314086504];
% cam1 = k1*[Rt1,t1'];
% 
% k2=[3310.400000 0.000000 316.730000; 0.000000 3325.500000 200.550000; 0.000000 0.000000 1.000000];
% Rt2 = [-0.28437648559126744000 0.95818717180552493000 0.03174117685280887500;...
%     -0.30447985736374050000 -0.05887271324279850400 -0.95069715013877931000;... 
%     -0.90907810612855156000 -0.28002026823733883000 0.30848995461910772000];
% t2 = [-0.0181396767412 -0.0277593369636 0.645855774902];
%     
% cam2 = k2*[Rt2,t2'];
% world = triangulate(x',y',cam1',cam2');
% scaled = world.*10000;
% figure
% plot(scaled(:,1),scaled(:,2))
% ptCld2 = pointCloud(scaled);
% figure
% pcshow(ptCld2)
% title('ptcld2')
% ptCld = pcmerge(ptCld,ptCld2,1);
% %%
% 
% %dinoringsparse 3,4
% img1 = imread('/home/semanti/Downloads/dinoSparseRing/dinoSR0003.png');
% img2 = imread('/home/semanti/Downloads/dinoSparseRing/dinoSR0004.png');
% figure
% imshow(img1);
% title('original img1');
% figure
% imshow(img2)
% title('original img2');
% %%
% % pin = [1578,1614;1578,1211;2057,1682;2021,1163]';
% % pout = [1577,1732;1585,1326;2066,1807;2033,1280]';
% % pin =[209,426;572,70;601,174;165,65];
% % pout=[206,432;570,68;602,182;167,65];
% % tform = estimateGeometricTransform(pin,pout,'projective');
% % tform=[  1.2486,  0.1631, -0.1511; -0.2465,  1.0720,  0.1027;0,0,1]
% tform=[0.9665, -0.1013,  0.0032;  0.0553,  1.3277, -0.1498;0,0,1]
% img1_bw = imbinarize(rgb2gray(img1),0.3);
% img2_bw = imbinarize(rgb2gray(img2),0.3);
% 
% xbl = 500;
% xbr = 2524;
% [x1,y1]=find(img1_bw==1);
% [x2,y2]=find(img2_bw==1);
% figure
% imshow(img1_bw)
% hold on;
% plot(y1,x1,'rx');
% title('points on image1 after segmentation');
% figure
% imshow(img2_bw);
% hold on;
% plot(y2,x2,'rx');
% title('points on image2 after segmentation');
% % aff = tform.T'
% % inp=[x1,y1]';
% %%
% % aff = tform.T'
% % aff = tform.T'
% aff=tform
% inp=[y1,x1]';
% pin = inp;
% figure
% imshow(img1);
% hold on;
% plot(pin(1,:),pin(2,:),'rx');
% title('points in img1')
% % hold on;
% % plot(1422,1486,'bd');
% % x=pin;
% x=inp;
% q = aff * [x; ones(1, size(x,2))];
% % q1 = aff * [1422;1486;1];
% p = q(3,:);
% y = [q(1,:)./p; q(2,:)./p];
% % p1 = q1(3,:);
% % y1 = [q1(1,:)./p1; q1(2,:)./p1];
% % for i=1:size(y_orig,2)
% %     
% %     if (y2(find(x2==y_orig(1))==y_orig(1))
% figure
% imshow(img2);
% hold on;
% % plot(pout(1,:),pout(2,:),'rx');
% % hold on;
% plot(y(1,:),y(2,:),'go');
% title('points in img2 mapped using affine')
% % hold on;
% % plot(y1(1,:),y1(2,:),'bd');
% %%
% 
% 
% k1=[3310.400000 0.000000 316.730000; 0.000000 3325.500000 200.550000; 0.000000 0.000000 1.000000];
% Rt1 = [-0.28437648559126744000 0.95818717180552493000 0.03174117685280887500;...
%     -0.30447985736374050000 -0.05887271324279850400 -0.95069715013877931000;... 
%     -0.90907810612855156000 -0.28002026823733883000 0.30848995461910772000];
% t1 = [-0.0181396767412 -0.0277593369636 0.645855774902];
%     
% cam1 = k1*[Rt1,t1'];
% 
% k2 =[3310.400000 0.000000 316.730000; 0.000000 3325.500000 200.550000; 0.000000 0.000000 1.000000];
% Rt2 = [-0.29401781384102649000 0.95254363718061841000 -0.07882994806514329000;...
%     0.10008332165785941000 -0.05134047099598639700 -0.99365293826058598000;...
%     -0.95054606727387558000 -0.30004129466721602000 -0.08023903828591291700];
% t2 = [-0.021187474226 -0.0187242689354 0.634951273531];
% cam2 = k2*[Rt2,t2'];
% world = triangulate(x',y',cam1',cam2');
% scaled = world.*10000;
% figure
% plot(scaled(:,1),scaled(:,2))
% ptCld3 = pointCloud(scaled);
% figure
% pcshow(ptCld3)
% title('ptcld3')
% ptCld = pcmerge(ptCld,ptCld3,1);
% 
% figure
% pcshow(ptCld)
% title('merged cld')