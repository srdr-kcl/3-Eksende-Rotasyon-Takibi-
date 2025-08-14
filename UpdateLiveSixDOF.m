function UpdateLiveSixDOF(anim, pos, R)

    o = pos(:)' ;
    X = R(:,1)' * anim.axisLength;
    Y = R(:,2)' * anim.axisLength;
    Z = R(:,3)' * anim.axisLength;
    
  
    set(anim.orgHandle, 'XData', o(1), 'YData', o(2), 'ZData', o(3));
    set(anim.quivXhandle, 'XData', o(1), 'YData', o(2), 'ZData', o(3), ...
        'UData', X(1), 'VData', X(2), 'WData', X(3));
    set(anim.quivYhandle, 'XData', o(1), 'YData', o(2), 'ZData', o(3), ...
        'UData', Y(1), 'VData', Y(2), 'WData', Y(3));
    set(anim.quivZhandle, 'XData', o(1), 'YData', o(2), 'ZData', o(3), ...
        'UData', Z(1), 'VData', Z(2), 'WData', Z(3));
    
   
    if ~isempty(anim.trailHandle)
        anim.trailData = [anim.trailData, o(:)]; 

       
        if size(anim.trailData, 2) > 1000 
            anim.trailData = anim.trailData(:, end-999:end);
        end

        
        if strcmp(anim.trailMode, 'dotsonly')
            set(anim.trailHandle, 'XData', anim.trailData(1,:), ...
                                  'YData', anim.trailData(2,:), ...
                                  'ZData', anim.trailData(3,:));
        elseif strcmp(anim.trailMode, 'all')
            set(anim.trailHandle, 'XData', anim.trailData(1,:), ...
                                  'YData', anim.trailData(2,:), ...
                                  'ZData', anim.trailData(3,:));
        end
    end
    
    
    axis equal;
    drawnow;
end