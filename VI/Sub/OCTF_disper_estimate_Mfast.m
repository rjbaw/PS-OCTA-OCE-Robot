function [M]=OCTF_disper_estimate_Mfast(hBdata,zCropRg,kmat,a)
% This calculates the contrast of the image and returns M, the lower the
% value of M, the higher the contrast is.
phaseV=kmat.*a;
Bdata=real(hBdata.*exp(1i*phaseV));
img=abs(fft(Bdata,[],1));
% p=max(img(zCropRg,:),[],1);M=1/sum(p(:)); %Method 1
% p=abs(diff(img(zCropRg,:),1));M=1/sum(p(:)); %Method 2
p=img(zCropRg,:)./sum(sum(img(zCropRg,:)));M = -sum(sum(p.*log1p(p)));
end