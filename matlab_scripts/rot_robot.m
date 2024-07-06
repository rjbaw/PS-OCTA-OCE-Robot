isdouble=-2;nX=-2;dep=-2;dz=-2;dRy=-2;
check = 'made it'

dz =-1;dR=-1;p1=-1;

slopes = zeros(n_frames,1);
intercepts = zeros(n_frames,1);

dep = size(img,1);
    
if dep >15
    
    for islice=1:n_frames

        slice = img(:,((islice-1)*width+1):islice*width);
        
        Stru =slice;
        % Stru =img';
        nX=size(Stru,2);
        gaussfilt = [1 1];
        thrsh_ratio = 0.98;
        BandLines=[5,12,95,100];
    %     Stru(1:150,:) = 0;
        figure(100+islice);imagesc(1*Stru);colormap(gray)
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
    
        hold on; plot(cc, 'r')
        hold on; plot(fittedCurve, 'b')
    
        angle_output = 9.65 * atan(p1);
        output_2 = fol / 51.2;
    
        slope = p1;
        intercept = fol;

        slopes(islice) = slope;
        intercepts(islice) = intercept;
    end
%

end

r1_slope = mean(slopes);
r1_intercept = mean(intercepts);
r2_slope = polyfit(round(linspace(1,500,3)),intercepts,1);
r2_slope = r2_slope(1);


r1_slope
r1_intercept
r2_slope

angle_output = 9.65 * atan(p1);
output_2 = fol / 51.2;

dz=(r1_intercept-210)/150/1000 * 1.2;
dR1=atan(r1_slope)*0.9
dR2=atan(r2_slope)*0.9