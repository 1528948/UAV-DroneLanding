clc;    % Clear the command window.
close all;  % Close all figures (except those of imtool.)
imtool close all;  % Close all imtool figures.
clear;  % Erase all existing variables.
workspace;
dir = '../matriz intrinseca\TOOLBOX_calib\m_intrinseca\n';
v = VideoReader("../../tomas/v_mesh.mp4");
v.CurrentTime = 20;
pan = readFrame(v);
th = 0.5;
%[h, w] = size(pan);
h = 1440;
w = 2560;
tic;
pan = imresize(pan,[h*th,w*th]);
params = mIntrin(dir);
toc;
while hasFrame(v)
    tic;
    f = imresize(undistortFisheyeImage(readFrame(v),params.Intrinsics),[h*th,w*th]);
    figure(21); imshow(f);
    toc;
    tic;
    pan = stitch(f,pan);
    figure(20);imshow(pan,[]);
    toc;
end
%Esta funcion la encontramos en internet, después de intentar durante horas
%hacer el mismo procedimiento con RANSAC
function [panorama] = stitch(image1, image2)
%stitching s rotovanim a s ciernymi okrajmi

%prevod obrazkov na sedotonovy
img1 = rgb2gray(image1);
img2 = rgb2gray(image2);

%velkosti obrazkov
[row1,col1] = size(img1);
[row2,col2] = size(img2);

%vyber vzoru zo stredu druheho obrazka
center = round(row2/2);
pattern = img2(center-20:center+20, 1:10);
imshow(pattern)

%hladanie korelacneho koeficientu medzi vzorom a prvym obrazkom
%pouzitim cross corelacie
coef = normxcorr2(pattern,img1);
max_coef = max(coef(:));

%prvotny uhol
angle = 0;

%porovnanie ci max z korelacnej matice je 1
%ak je 1 znamena, ze obrazok nie je natoceny, ak nie je jedna hladanie uhla
if (abs(1-max_coef)>0.0001) %podmienka ci je koeficient je 1 alebo nie s urcitou presnostou
    all_angles = [1:1:30 -(1:1:30)]; %skusobne uhly od -30 do 30
    len = length(all_angles);
    %rotovanie prveho obrazka podla kazdeho uhla
    for (i=1:len) 
        img = imrotate(img1, all_angles(i)); 
        coef = normxcorr2(pattern,img);
        max_coef = [max_coef max(coef(:))]; %ukladanie maxim z korelacnej matice pre kazdy uhol
    end
    index = find(max_coef==max(max_coef(:))); %najdenie indexu, pre ktory korelacia bola najvacsia
    angle = all_angles(index);
    image1 = imrotate(image1, angle); %natocenie povodneho obrazka podla najvhodnejsieho uhla
    img1 = rgb2gray(image1); %prevod povodneho obrazka na sedotonovy
    imshow(img1)
    [row1,col1] = size(img1);
    coef = normxcorr2(pattern,img1); %korelacie medzi vzorom a uz otocenym prvym obrazkom
end

%najdenie indexov z korelacnej matice, kde je korelacny koeficient
%najvacsi
[ypeak, xpeak] = find(coef==max(coef(:)));

%najdenie indexov kde sa zacina vzor v prvom obrazku
yoffSet = ypeak-size(pattern,1);
xoffSet = xpeak-size(pattern,2);

%zistenie ci sa v prvom obrazku nachadza horny alebo dolny roh druheho
%obrazka a zaroven index horneho alebo dolneho rohu
%(respektive ci druhy obrazok sa nachadza vyssie alebo nizsie alebo je v
%rovnakej vyske)
flag = 0;
up = row2 - center - 20; %horna zvysna cast druheho obrazku
down = row2 - up; %dolna zvysna cast prveho obrazku
if (yoffSet - up < 0) %ci sa nachadza dolny roh
    flag = 'down';
    yoffSet = yoffSet + down; %y-ovy index dolneho rohu
elseif (yoffSet - up == 0)
    flag = 'same'; %suradnice sa nemenia, obrazky su v rovnakej vyske
else
    flag = 'up'; %horny roh
    yoffSet = yoffSet - up; %index horneho rohu
end

%vlozenie do obrazka podla rovnakeho suradnicoveho systemu 
border = 0; %obrazky su posunute kvoli ciernym pixelom, preto ich treba vynulovat
if (strcmp(flag, 'down'))
    if (angle > 0)
        [r1, c1] = find(img1, 1,'first'); %indexy bodu A pomocou prveho nenuloveho prvku v img1
        border = r1;
    end
    if (angle < 0)
       temp = rot90(img1); %pootocenie img1 o 90stupnov v protismere hodiniek
       [r2, c2] = find(temp, 1,'first');
       temp = double(temp);
       temp(r2, c2) = 1000; %prepis prveho prvku na 1000 na oznacenie 
       temp = rot90(temp, -1); %spatna rotacia
       [r2, c2] = find(temp==1000); %indexy B 
       border = r2; %posunutie, hranica ktora je tovrena ciernymi pixelmi
    end
    xWorldLimits = [1 col1+1];
    yWorldLimits = [row1-yoffSet+1-border row1-yoffSet+row1+1-border];
    R1 = imref2d(size(img1), xWorldLimits, yWorldLimits); %suradnicovy system pre img1
    
    xWorldLimits = [xoffSet+1 xoffSet+col2+1];
    yWorldLimits = [1 row2+1];
    R2 = imref2d(size(img2),xWorldLimits,yWorldLimits); %suradnicovy system pre img2
elseif (strcmp(flag, 'up'))
    xWorldLimits = [1 col1+1];
    yWorldLimits = [1 row1+1];
    R1 = imref2d(size(img1), xWorldLimits, yWorldLimits);
    
    xWorldLimits = [xoffSet+1 xoffSet+col2+1];
    yWorldLimits = [yoffSet+1 yoffSet+row2+1];
    R2 = imref2d(size(img2),xWorldLimits,yWorldLimits);
elseif (strcmp(flag,'same'))
    xWorldLimits = [1 col1+1];
    yWorldLimits = [1 row1+1];
    R1 = imref2d(size(img1), xWorldLimits, yWorldLimits);
    
    xWorldLimits = [xoffSet+1 xoffSet+col2+1];
    yWorldLimits = [1 row2+1];
    R2 = imref2d(size(img2),xWorldLimits,yWorldLimits);
end

[pano, RP] = imfuse(img1,R1,img2,R2); %spojenie do jedneho suradnicoveho systemu v sedotonovom
imshow(pano)

%vlozenie originalnych obrazkov do vyslednej panoramy
if (strcmp(flag, 'down'))
    panorama = zeros(size(pano));
    panorama(row1-yoffSet+1-border:row1-yoffSet+row1-border, 1:col1, :) = image1;
    panorama(1:row2, xoffSet+1:xoffSet+col2, :) = image2;
elseif (strcmp(flag, 'up'))
    panorama = zeros(size(pano));
    panorama(1:row1, 1:col1, :) = image1;
    panorama(yoffSet+1:yoffSet+row2, xoffSet+1:xoffSet+col2, :) = image2;
elseif (strcmp(flag,'same'))
    panorama = zeros(size(pano));
    panorama(1:row1, 1:col1, :) = image1;
    panorama(1:row2, xoffSet+1:xoffSet+col2, :) = image2;
end

%pretypovanie vyslednej panoramy
panorama = uint8(panorama);

end

function params = mIntrin(dir)
    images = imageDatastore(dir);
    [imagePoints,boardSize] = detectCheckerboardPoints(images.Files);
    squareSize = 32; % millimeters
    worldPoints = generateCheckerboardPoints(boardSize,squareSize);
    I = readimage(images,numel(images)); 
    imageSize = [size(I,1) size(I,2)];
    params = estimateFisheyeParameters(imagePoints,worldPoints,imageSize);
end