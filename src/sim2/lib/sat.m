function res = sat(x, e)
    
    if x > -e && x < e
        res = x / e;
        return;
    end
    
    if x >= 0
        res = 1;
    else 
        res = -1;
    end
end
        