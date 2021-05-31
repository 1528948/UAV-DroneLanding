clearvars,
close all,
clc,
%ima = imread('tomas/fotos_aruco/12.jpg');
v = VideoReader("tomas/v_aruco.mp4");
v.CurrentTime = 13;
i=1;
frame = 5;
fps = 25;

height = 100;
while hasFrame(v)
    if (mod(i,frame)==0)
        ima = readFrame(v);
        
        %figure(6)
        %h = histogram(ima);
        % Aplicamos un threshold para aislar el negro del marcador ArUco
        tima = im2uint8(ima < 240);
        figure(1);subplot(2,2,1);imshow(ima);
        title('original');
        %figure(2)
        %imshow(tima);
        % Convertimos la imagen es escala de grises
        gima = rgb2gray(tima);
        %figure(3)
        %imshow(gima);
        % Procedemos a definir un trazado de contorno de los marcadores
        %figure(4);
        %cima = imcontour(gima,1); %1 nivel de curva para eliminar ruido innecesario

        %rellenamos los huecos del marcador para posteriormente extraer las esquinas
        %y no tener exceso de esquinas con los cuadrados interiores
        fima = imfill((255 - gima),'holes');
        fima = 255-(gima-(255-fima)); %ARUCO SOLO 

        se = strel('rectangle',[4 4]);
        se2 = strel('rectangle',[7 7]);
        fima = imerode(imdilate(fima,se),se2);
        %subplot(1,2,1);imshow(fima);
        xima = 255-(imfill((255 - fima),'holes')); %ARUCO NEGRO
        %detectamos las 4 esquinas del marcador
        %subplot(1,2,2);imshow(xima);
        corners = detectHarrisFeatures(xima,'MinQuality',0.1);

        %DETECTAR ID ARUCO 

        %Obtenemos las coordenadas del cuadrado (marcador)
        coor = corners.Location;
        coor = rmoutliers(coor); %eliminar outliers
        xmin = min(coor(:,1)); xmax = max(coor(:,1));
        ymin = min(coor(:,2)); ymax = max(coor(:,2));

        %ROTACIÓN ARUCO
        es1 = coor(find(coor(:,1)==xmin),:);
        es2 = coor(find(coor(:,2)==ymin),:);
        es3 = coor(find(coor(:,1)==xmax),:);
        es4 = coor(find(coor(:,2)==ymax),:);
        es = [es1;es2;es3;es4];
        figure(1);subplot(2,2,2);imshow(fima);
        hold on;
        plot(es1(1),es1(2),'.','Color','r','MarkerSize',20)
        plot(es2(1),es2(2),'.','Color','r','MarkerSize',20)
        plot(es3(1),es3(2),'.','Color','r','MarkerSize',20)
        plot(es4(1),es4(2),'.','Color','r','MarkerSize',20)
        title('esquinas');
        hold off;

        %angle = 180 + atan2((es2(2) - es1(2)),(es2(1) - es1(1))) * (180 / pi);
        %disp(angle)
        %rima = imrotate(fima,angle);


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% HOMOGRAFÍA CON MATRIZ INTRÍNSECA A PARTIR DE IMÁGENES (NO UTILIZADO)
        %images = imageDatastore('tomas/m_intrinseca');
        %[imagePoints,boardSize] = detectCheckerboardPoints(images.Files);
        %squareSize = 32; % millimeters
        %worldPoints = generateCheckerboardPoints(boardSize,squareSize);
        %I = imread('tomas/fotos_aruco/1.jpg');
        %imageSize = [size(I,1) size(I,2)];
        %params = estimateFisheyeParameters(imagePoints,worldPoints,imageSize);
        %J1 = undistortFisheyeImage(I,params.Intrinsics);
        %figure(4)
        %imshow(J1)
        %figure(1)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


        %DETECTAR ESQUINAS NUEVA IMAGEN ROTADA
        %corners = detectHarrisFeatures(imrotate(xima,angle),'MinQuality',0.1);
        %subplot(1,3,1);imshow(rima); 
            %hold on;
            %plot(corners);
            %hold off;
            %title('esquinas');
        %coor = corners.Location;
        %coor = rmoutliers(coor);

        %xmin = min(coor(:,1)); xmax = max(coor(:,1));
        %ymin = min(coor(:,2)); ymax = max(coor(:,2));
        %{
        [s,j] = size(coor(:,1));

        left_t = coor(:,1) < xmin+20;
        right_t = coor(:,1) > xmax-10;
        left = [];
        right = [];
        for i = 1:s
            if right_t(i) == 1
                right = cat(1,right,coor(i,:));
            end
            if left_t(i) == 1
                left = cat(1,left,coor(i,:));
            end
        end

        right = sortrows(right,2);

        subplot(2,2,2);imshow(rima); 
        hold on;
        q1 = left(1,:);
        q2 = left(end-1,:);
        q3 = right(1,:);
        q4 = right(end,:);

        plot(left(:,1),left(:,2),'o','Color','b')
        plot(right(:,1),right(:,2),'o','Color','b')
        plot(q1(1),q1(2),'.','Color','r','MarkerSize',20)
        plot(q2(1),q2(2),'.','Color','r','MarkerSize',20)
        plot(q3(1),q3(2),'.','Color','r','MarkerSize',20)
        plot(q4(1),q4(2),'.','Color','r','MarkerSize',20)
        hold off;
        title('esquinas');
        %}



        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % HOMOGRAFÍA
        l_aruco = 194; %Longitud aruco mm 194
        % These are the points in the image
        pa = [es1(1) es1(2)]';
        pb = [es2(1) es2(2)]';
        pc = [es3(1) es3(2)]';
        pd = [es4(1) es4(2)]'; 
        % Refence Points from Court Model
        n = 100;
        A = [n n]'; 
        B = [n+l_aruco n]';
        C = [n+l_aruco n+l_aruco]';
        D = [n n+l_aruco]';
        pin = [pa pb pc pd]; % 2xN matrix of inputs
        pout = [D C B A]; % 2xN matrix of output
        pout(2,:)=500-pout(2,:); %flip to agree with pin
        H = fitgeotrans(pin',pout','projective');
        [rima_w, ref] = imwarp(255-fima,H,'OutputView',imref2d(size(fima)));
        [xima_w, ref] = imwarp(255-xima,H,'OutputView',imref2d(size(xima)));
        rima_w=255-rima_w;
        xima_w=255-xima_w;

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        corners = detectHarrisFeatures(xima_w,'MinQuality',0.1);
        coor = corners.Location;
        coor = rmoutliers(coor);
        xmin = min(coor(:,1)); xmax = max(coor(:,1));
        ymin = min(coor(:,2)); ymax = max(coor(:,2));

        esq1 = [xmin ymin];
        esq2 = [xmax ymin];
        esq3 = [xmax ymax];
        esq4 = [xmin ymax];

        figure(1);subplot(2,2,3);imshow(rima_w);
        hold on;
        plot(esq1(1),esq1(2),'.','Color','r','MarkerSize',20)
        plot(esq2(1),esq2(2),'.','Color','r','MarkerSize',20)
        plot(esq3(1),esq3(2),'.','Color','r','MarkerSize',20)
        plot(esq4(1),esq4(2),'.','Color','r','MarkerSize',20)
        title('esquinas homografía');
        hold off;

        %Cortamos la imagen para que solo aparezca el ArUco
        cima = imcrop(rima_w,[xmin ymin xmax-xmin ymax-ymin]); %Cortamos la imagen antes de rellenar huecos
        %disp([xmin ymin xmax-xmin ymax-ymin])
        bima = imbinarize(cima);

        nsquares = 7; %numero de cuadrados del ArUco
        th = 0.6;

        [w,h] = size(bima);
        matrix = [];
        sqx = int64(w/nsquares);
        sqy = int64(h/nsquares);
        jima = imcrop(bima,[sqx sqy w-sqx*2 h-sqy*2]);
        [w,h] = size(jima);
        %subplot(1,4,3);imshow(jima);title('aruco crop');
        for x = 0:sqx+3:w %retocar,falla
            fil = [];
            for y = 0:sqy+3:h
                maj = jima((int8(x+sqx*(1-th))):(int8(x+sqx*th)),(int8(y+sqy*(1-th))):(int8(y+sqy*th)));
                fil(end+1) = mode(mode(maj));
            end
            matrix(end+1, :) = fil; 
        end

        %Calcular ID
        id_con = [192];
        id=0;
        r=0;
        while (~ismember(id,id_con) && r<360)
            bin = [];
            if (r~=0)
                matrix = rot90(matrix);
            end
            for i = 1:1:nsquares-2
                bin(end+1) = matrix(i,2);
                bin(end+1) = matrix(i,4);
            end
            id = bi2de(bin,'left-msb');
            r = r+90;
        end
        if (r==360)
            r = r-90;
        end

        if (ismember(id,id_con))
            % CALCULAR ALTURA DRON
            h1 = imread('tomas/fotos_aruco/h1.jpg');
            h2 = imread('tomas/fotos_aruco/h2.jpg');
            h_1 = 500;
            h_2 = 1000;
            height = 0;
            %figure(2)
            %subplot(1,2,1); imshow(h1);title('h1 500mm');
            %subplot(1,2,2); imshow(h2);title('h2 1000mm');
            h1_p1 = [1106 595];
            h1_p2 = [1601 595];
            h2_p1 = [1215 606];
            h2_p2 = [1467 593];
            l = sqrt((es1(1)-es2(1))^2+(es1(2)-es2(2))^2);
            l1 = sqrt((h1_p1(1)-h1_p2(1))^2+(h1_p1(2)-h1_p2(2))^2);
            l2 = sqrt((h2_p1(1)-h2_p2(1))^2+(h2_p1(2)-h2_p2(2))^2);

            %equacion recta
            m = -0.21;
            n = 151.99;
            height = m*l+n;

            cima = imrotate(cima,r);
            [w,h] = size(cima);
            figure(1);subplot(2,2,4);imshow(cima);title('ArUco');
            text(w/2,h/2,'ID='+string(id),'Color','red','FontSize',14,'HorizontalAlignment','center');
            text(w/2,h/2+30,'height='+string(height)+'cm','Color','red','FontSize',14,'HorizontalAlignment','center');
            speed = (height2-height)*((1/frame)/fps);
            text(w/2,h/2+60,'speed='+string(speed)+'cm/s','Color','red','FontSize',14,'HorizontalAlignment','center');
        end
    end
    i = i+1;
    height2 = height;
end






