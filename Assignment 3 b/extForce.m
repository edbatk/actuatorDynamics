function [F] = extForce(t,X,c,p)
    F = c.toolAmp*sin(t.*c.toolFreq);
end