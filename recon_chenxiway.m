%church 1,2
close all;
clc;
clear;

%%
img1 = imrotate(imread('/home/semanti/Downloads/buildings/Tower1_p1.jpg'),-90);
img2 = imrotate(imread('/home/semanti/Downloads/buildings/Tower1_p2.jpg'),-90);
% C1=corner(img1);
Data = struct;
% C2=corner(img2);
% img1 = imread('/home/semanti/Documents/cnngeometric_pytorch/source.png');
% img2 = imread('/home/semanti/Documents/cnngeometric_pytorch/target.png');
figure
imshow(img1);
title('original img1');
hold on;
% plot(C1(:,1),C1(:,2),'gx');
figure
imshow(img2);
title('original img2');
%%
% pin = [1578,1614;1578,1211;2057,1682;2021,1163]';
% pout = [1577,1732;1585,1326;2066,1807;2033,1280]';
% pin =[209,426;572,70;601,174;165,65];
% pout=[206,432;570,68;602,182;167,65];
% tform = estimateGeometricTransform(pin,pout,'projective');
% tform=[  1.2486,  0.1631, -0.1511; -0.2465,  1.0720,  0.1027;0,0,1]
% tform=[ 1.0343, -0.0020,  0.0027; -0.0123,  1.0664, -0.1928;0,0,1]%cropped
pt1=[1991 829;1973 991;2021 985;1895 1206;2129 1194;1877 1278;2141 1284;1842 1427;2183 1421;1824 2217;2218 2217;...
    1710 2438;2362 2444;1626 2594;1632 2713;2488 2719;2458 2588];
pt2 = [1035 789;1011 975;1065 963;915 1179;1131 1191;897 1269;1149 1281;849 1413;1185 1407;825 2193;1227 2193;705 2433;...
    1347 2451;585 2601;573 2697;1431 2709;1413 2577];
pt1 = pt1(randperm(size(pt1,1)),:);
pt2 = pt2(randperm(size(pt2,1)),:);
tform=[ 0.7099, -0.0671,  0.4236;  0.0255,  0.9978,  0.0296;0,0,1];
img1_bw = imbinarize(rgb2gray(img1),0.3);
img2_bw = imbinarize(rgb2gray(img2),0.3);
% corners=detectHarrisFeatures(img1_bw);
% figure
% imshow(img1);
% title('original img1');
% hold on;
% plot(corners.selectStrongest(50));
% plot(C1(:,1),C1(:,2),'gx');

%%
% img1_bw=img1_bw(60:180,60:180);
% img2_bw=img2_bw(60:180,60:180);
% img1=img1(60:180,60:180);
% img2=img2(60:180,60:180);
% xbl = 500;
% xbr = 2524;
% [l,w]=size(img1_bw);
% [x1,y1]=find(img1_bw==0);
% [x2,y2]=find(img2_bw==0);
% % indx = find(x1>60 & x1<200);
% % indy = find(y1>60 & y1<200);
% % indx = find(x1>756 & x1<2268);
% % indy = find(y1>756 & y1<2268);
% indx = find(x1>300 & x1<3000);
% indy = find(y1>300 & y1<2500);
% ind = intersect(indx,indy);
x1=pt1(:,1);%x1(ind);
y1=pt1(:,2);%y1(ind);
figure
imshow(img1);
hold on;
plot(x1,y1,'rx');
title('points on image1 ');
% indx = find(x2>30 & x2<180);
% indy = find(y2>30 & y2<180);
% indx = find(x2>756 & x2<2268);
% indy = find(y2>756 & y2<2268);
% indx = find(x2>300 & x2<3000);
% indy = find(y2>300 & y2<2500);
% ind = intersect(indx,indy);
%%
x2=pt2(:,1);%x2(ind);
y2=pt2(:,2);%y2(ind);
figure
imshow(img2);
hold on;
plot(x2,y2,'bx');
title('points on image2 after segmentation');
% aff = tform.T'
% inp=[x1,y1]';
%%
% aff = tform.T'
% aff = tform.T'
% aff=inv(tform);
aff=tform;
% aff= [1 0 10;0 1 0;0 0 1]
% inp=[y1,x1];
% bound_img1 = boundary(inp(:,1),inp(:,2));
% inp=[x1,y1]';
pin = pt1;%inp(bound_img1,:);
figure
imshow(img1);
hold on;
plot(pin(:,1),pin(:,2),'rx');
title('points in img1')
% hold on;
% plot(1422,1486,'bd');
% x=pin;
% x=[x1,y1]';
x=pin';
% x=[y1,x1]';
q = aff * [x; ones(1, size(x,2))];
% q1 = aff * [1422;1486;1];
p = q(3,:);
y = [q(1,:)./p; q(2,:)./p];
% y(1,:)=(y(1,:)./240).*3024;
% y(2,:)=(y(2,:)./240).*4032;
% p1 = q1(3,:);
% y1 = [q1(1,:)./p1; q1(2,:)./p1];
% for i=1:size(y_orig,2)
%     
%     if (y2(find(x2==y_orig(1))==y_orig(1))
%%
figure
imshow(img2);
hold on;
% plot(pout(1,:),pout(2,:),'rx');
% hold on;
plot(y(1,:),y(2,:),'bo');
% plot(y(1,:),y(2,:),'go')
title('points in img2 mapped using affine')
% disp('k')
% hold on;
% plot(y1(1,:),y1(2,:),'bd');
%%
% bound_orig_ind  = boundary(y2,x2);
orig_img2_pts = pt2;%[y2,x2];
% bound_orig2 = orig_img2_pts(bound_orig_ind,:);
% aff_tr = [(y(2,:))',(y(1,:))'];
aff_tr = y';
c1 = [mean(x2),mean(y2)];
c2 = [mean(y(1,:)),mean(y(2,:))];
D=c1-c2;
aff_tr_trans = [aff_tr(:,1)+D(1),aff_tr(:,2)+D(2)];
figure
imshow(img2);
hold on;
plot(x2,y2,'rx');
hold on;
plot(aff_tr_trans(:,1),aff_tr_trans(:,2),'gx');

%%
% bound_orig  = bound_orig_ind;
% bound_aff_t =aff_tr_trans; % boundary(aff_tr_trans(:,1),aff_tr_trans(:,2));
% figure
% imshow(img2);
% hold on;
% plot(bound_orig2(:,1),bound_orig2(:,2),'rx');
% hold on;
% plot(aff_tr_trans(:,1),aff_tr_trans(:,2),'gx')

%%
% hold on;
% plot(y2,x2,'rx');
boundpts_orig = pt2;
boundpts_aff = aff_tr_trans;%[aff_tr_trans(bound_aff_t,1),aff_tr_trans(bound_aff_t,2)];
dist = pdist2(boundpts_orig,boundpts_aff);
[v,ind]=min(dist,[],2);
all_img1 = x';
b_img1 = x';%[all_img1(bound_aff_t,1),all_img1(bound_aff_t,2)];
% bound_img1 = all_img1(bound_aff_t,:);
% bound_img2 = boundpts_orig(ind,:);
bound_img1 = b_img1(ind,:);
bound_img2 = boundpts_orig;
figure
imshow(img1);
hold on;
plot(bound_img1(:,1),bound_img1(:,2),'rx');
figure
imshow(img2);
hold on;
plot(bound_img2(:,1),bound_img2(:,2),'bx');
%%
    translationOfCamera2 = [70, 0, 0];
%     translationOfCamera2 = [5.6, 0, 0];
    
% % translationOfCamera2 = [0.80,0,0];
% % translationOfCamera2 = [2.5, 0, 0];
R = eye(3);% rotationOfCamera2, align with the wolrd coordinate.
T1 = [0 0 0];
T2 = translationOfCamera2;% translationOfCamera2.
extrinsicMtx1 = [R;T1];
extrinsicMtx2 = [R;T2];
% 
% opticalCenter = [500 600];
% fx = 4; fy = 4;
% sx = 1/14; sy = 1/14;
% cx = 2000; cy = 1500;
% % cx=159;cy=119;
% % cx=502;cy=424
% intrinsicMtx = [fx/sx, 0, 0; 0, fy/sy, 0; cx, cy, 1];
intrinsicMtx = [583.141567003901,0,0;0,584.782897695379,0;2150.03791355007,1287.58122795129,1];
cam1 = intrinsicMtx'*[R,T1'];
cam2 = intrinsicMtx'*[R,T2'];
% % cam1=cam1';
% % cam2=cam2';
% % k1=[3310.400000 0.000000 316.730000; 0.000000 3325.500000 200.550000; 0.000000 0.000000 1.000000];
% % Rt1 = [-0.14396457836077139000 0.96965263281337499000 0.19760617153779569000;...
% %     -0.90366580603479685000 -0.04743335255026152200 -0.42560419233334673000;...
% %     -0.40331536459778505000 -0.23984130575212276000 0.88306936201487163000];
% % t1 = [-0.010415508744 -0.0294278883669 0.673097816109];
% %     
% % cam1 = k1*[Rt1,t1'];
% % k2 =[3310.400000 0.000000 316.730000; 0.000000 3325.500000 200.550000; 0.000000 0.000000 1.000000];
% % Rt2 = [-0.23143687262851170000 0.96422332027030622000 0.12926937165558827000;...
% %     -0.65860811165749811000 -0.05749426020475298400 -0.75028640486482034000;...
% %     -0.71601208187197274000 -0.25878137856376976000 0.64835049619606955000];
% % t2 = [-0.0143392676288 -0.0315356332107 0.659314086504];
% % cam2 = k2*[Rt2,t2'];
% ind=find(unique(bound_img1));
% p1=bound_img1(ind,:);
% p2=bound_img2(ind,:);
p1=[bound_img1(:,1),bound_img1(:,2)];
p2=[bound_img2(:,1),bound_img2(:,2)];
% DT1 = delaunayTriangulation(p1);
uv1=p1;
uv2=p2;
% Setup world coordinate
translationOfCamera2 = [70, 0, 0];
baseLine = 70;
focalLength = 583;
xy1 = uv1;
xy2 = uv2;
cx = 2150; cy = 1287;
for i = 1:size(uv1,1)
    disparity(i) = xy2(i,1)-xy1(i,1);
    Z(i) = baseLine*focalLength/disparity(i);
    X(i) = baseLine*(xy1(i,1) + xy2(i,1)-2*cx)/(2*disparity(i));
    Y(i) = baseLine*(xy1(i,2) + xy2(i,2)-2*cy)/(2*disparity(i));
end

figure,
plot3(X, Y, Z, 'b.')
axis equal

%% Find connection

% tri = getConnection(X, Y, p1);
% tri([1,2,3,4,5,8,21,23],:) = [];
% tri = [tri; [32,33,34;34,35,36;36,37,38;38,39,40]];
% tri([1,4,5,10,11,14,16,18,19,21,22,23,26,27,28,32,43,45:48,50,51,52,55:58],:) = [];
% tri = [tri; 8,9,10; 10,11,12;12,13,14;14,15,16;16,17,18;18,19,20;20,21,22;27,28,29;29,30,31;31,32,33;33,34,35;35,36,37;37,38,39;39,40,41];

tri = delaunay(X,Y);
poly1 = polyshape(X(1:17), Y(1:17));
delete_list = [];
partial_list = [];
for i = 1:length(tri)
    idx = tri(i,:);
    poly2 = polyshape([X(idx(1)), X(idx(2)), X(idx(3))], [Y(idx(1)), Y(idx(2)), Y(idx(3))]);
    overlap = intersect([poly1 poly2]);

    if ~overlap.NumRegions 
        delete_list = [delete_list, i]; 
    else
        if abs(area(poly2)-area(overlap))>1e-10
            partial_list = [partial_list, i];
        end
    end
end
unconnect_idx = unique(tri(partial_list,:));
tri([delete_list,partial_list],:) = [];
% 
% test1 = polyshape(X(unconnect_idx(1:3)), Y(unconnect_idx(1:3)));
% over1 = intersect(poly1, test1);
% test2 = polyshape(X(unconnect_idx(2:4)), Y(unconnect_idx(2:4)));
% over2 = intersect(poly1, test2);
% 
% if area(over1)<1e-10
%     start1 = 2;
% else
%     if area(over2)<1e-10
%         start1 = 1;
%     end
% end
% 
% add_tri = [];
% while start1<length(unconnect_idx)
%     add_tri = [add_tri, unconnect_idx(start:start+2)];
%     start1 = start1 +2;
% end
%     
% add_tri = add_tri';
% 
% tri = [tri; add_tri];

pts = [X', Y', Z'];
faces = tri;

for i = 1:7
[pts, faces] = myLoopSubdivision(pts, faces);
end

figure,
plot3(pts(:,1), pts(:,2), pts(:,3),'.')
axis equal

affine_mtx = estimateGeometricTransform(data1,data2,'affine');
project_mtx = estimateGeometricTransform(data1,data2,'projective');

Data.Ver = pts;
Data.Tri = faces;
Data.Fpts = [X',Y',Z'];
Data.baseline = 70;
Data.img1 = img1;
Data.img2 = img2;
Data.uv1 = data1;
Data.uv2 = data2;
Data.affine_mtx = affine_mtx.T;
Data.project_mtx = project_mtx.T;

save_file = ['\buildings\',type,num2str(num),'.mat'];
% 
% if issave
%     fprintf('The .mat file is saved!')
%     save([Path,save_file], 'Data')
% end






DT2 = delaunay(p2);
% [ newVertices, newFaces ]=LoopSubdivision(p1,DT1);
% [ newVertices2, newFaces2 ]=LoopSubdivision(p2,DT2);
% np1=newVertices;
% np2=newVertices2(1:size(np1,1),:);
% world = triangulate(x',y',cam1',cam2');
% n1=[];
% n2=[];
% for i=1:9
%     for j=1:length(p1)-1
%         mid=(p1(j,:)+p1(j+1,:))./2;
%         n1=[n1;p1(j,:)];
%         n1=[n1;mid];
%     end
%     n1=[n1;p1(j,:)]
%     p1=n1;
%     for k=1:length(p2)-1
%         mid2=(p2(k,:)+p2(k+1,:))/2;
%         n2=[n2;p2(k,:);mid2];
%     end
%     n2=[n2;p2(k,:)]
%     p2=n2;
% end
% [np1,np2]=loopsubdivision(p1,p2);       
%  world = triangulate(np1,np2,cam1',cam2');   
world = triangulate(p1,p2,cam1',cam2');
% [k,av]=convhull(world);
% DT1 = delaunay(world);
[ newVertices, newFaces ]=LoopSubdivision(world,DT2);
% [k,av]=convhull(world);
[ newVertices, newFaces ]=LoopSubdivision(newVertices,newFaces);
[ newVertices, newFaces ]=LoopSubdivision(newVertices,newFaces);
[ newVertices, newFaces ]=LoopSubdivision(newVertices,newFaces);
[ newVertices, newFaces ]=LoopSubdivision(newVertices,newFaces);
% [ newVertices, newFaces ]=LoopSubdivision(newVertices,newFaces);
scaled=newVertices;
% world = triangulate(p1,p2,cam1',cam2');
% ty=[y(2,:);y(1,:)]';
% tx=[x(2,:);x(1,:)]';
% world = triangulate(ty,tx,cam2',cam1');
% world = triangulate(y',x',cam2',cam1');
% scaled = world.*10000;
% scaled = world;
figure
% plot(scaled(:,1),scaled(:,2))
ptCld1 = pointCloud(scaled);
% ptCld1 = pointCloud(world);
% figure
pcshow(ptCld1)
% figure
% trimesh(newFaces,newVertices(:,1),newVertices(:,2))
% title('ptcld1')
% axis tight;