isdouble=-2;nX=-2;dep=-2;dz=-2;dRy=-2;
check = 'made it'

dz =-1;dR=-1;p1=-1;

try
    Stru =img(:,1:500);
    nX=size(Stru,2);
    dep = size(Stru,1);
catch
    dep = -1;
end
    
if dep >15
    gaussfilt = [1 1];
    thrsh_ratio = 0.98;
    BandLines=[5,12,95,100];
    %crop the top 150 rows
    Stru(1:40,:) = 0;

    %figure(100);imagesc(1*Stru);colormap(gray)
    
    cimgc=imgaussfilt(Stru,gaussfilt);
    cc = zeros(1,nX);
    cimgMax=max(cimgc);

    for iX=1:nX
        vol=cimgc(:,iX);
        loc = find(vol>cimgMax(iX)*thrsh_ratio,1,'first');
            while isempty(loc)
                cimgMax(iX)=cimgMax(iX)*thrsh_ratio;
                loc = find(vol>cimgMax(iX)*thrsh_ratio,1,'first');
            end
            cc(iX)=loc;
    end
    cc=round(medfilt1(cc,56));

    stdcc = std(cc);
    p = polyfit(1:length(cc),cc,1);
    p1 = p(1);
    p2 = p(2);
    fol = mean(cc);
    fittedCurve = polyval(p,1:length(cc));
    stdccN = 1-stdcc/50;

    xin=1:1:100;

    %hold on; plot(cc, 'r')
    %hold on; plot(fittedCurve, 'b')

    angle_output = 9.65 * atan(p1);
    output_2 = fol / 51.2;
    dx=0;dy=0;dz=(fol-180)/150/1000 * 1.2;
    %dRx=0;dRy=atan(p1)*0.2913;dRz=0;
    dR=atan(p1)*0.9
% 
end