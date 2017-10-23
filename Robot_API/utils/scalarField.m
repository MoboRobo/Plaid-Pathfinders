classdef scalarField < handle
    %scalarField Implements a (discretized) field over a finite rectangle 
    % in the plane. Maps a rectangle onto an array and permits accesses to
    % the array based on continuous (x,y) coordinates. Establishes the 
    % relationship between discrete and continuous coordinates at construction
    % time and remembers it thereafter. Therefore, it is suitable for 
    % writing to a file and reading it back again without having to
    % remember the transform externally. The special value "empty" can be
    % used to test if a cell was never written.
    %
    % Author: Al Kelly. 
    
    properties(Constant)
        empty = inf;
    end
    
    properties(Access = private)

    end
    
    properties(Access = public)
        xMin;
        xMax;
        numX;
        yMin;
        yMax;
        numY;
        dx;
        dy;
        cellArray = [];
    end
    
    methods(Static = true)
        
    end
    
    methods(Access = private)
         
    end
            
    methods(Access = public)
        
        function obj = scalarField(xMin,xMax,numX,yMin,yMax,numY)
            % Constructs a scalar field on the supplied rectangle. The
            % underlying array has numX samples in X and numY samples in Y. 
            if(nargin == 6)
                obj.xMin = xMin;
                obj.xMax = xMax;
                obj.numX = numX;
                obj.yMin = yMin;
                obj.yMax = yMax;
                obj.numY = numY;
                obj.dx = (xMax-xMin)/numX;
                obj.dy = (yMax-yMin)/numY;
                obj.cellArray = obj.empty*ones(numX,numY);
            end
        end
        
        function setAll(obj,value)
            % Set all cells to some value;
            for i=1:obj.numX
                for j=1:obj.numY
                    obj.cellArray(i,j) = value;
                end
            end
        end
        function set(obj,x,y,val)
            % Set the cell containing (x,y) to value.
            if(~obj.isInBounds(x,y))
                err = MException('scalarField:Out Of Bounds', ...
                    'Exiting...');
                throw(err);
            end
            i = obj.xToi(x);
            j = obj.yToj(y);
            obj.cellArray(i,j) = val;
        end
        
        function val = get(obj,x,y)
            % return the value in the cell that contains (x,y)
            if(~obj.isInBounds(x,y))
                err = MException('scalarField:outOfbounds', ...
                    'Out of Bounds Lookup in Scalar Field');
                throw(err);
            end
            i = obj.xToi(x);
            j = obj.yToj(y);
            val = obj.cellArray(i,j);
        end
        
        function val = isInBounds(obj,x,y)
            % Returns true if (x,y) is inside the field boundaries
            
            if(x<obj.xMin || x>obj.xMax || y<obj.yMin || y>obj.yMax)
                val = false;
            else
                val = true;
            end
        end
        
        function [min, max] = range(obj)
            % Return maximum and minimum non infinite values in the field.
            min = inf ; max = -inf;
            for i=1:obj.numX
                for j=1:obj.numY
                    if(isinf(obj.cellArray(i,j))); continue; end;
                    if(obj.cellArray(i,j) < min) 
                        min = obj.cellArray(i,j); 
                    end;
                    if(obj.cellArray(i,j) > max) 
                        max = obj.cellArray(i,j); 
                    end;
                end
            end
        end
        
        function points = getNeighbors(obj,pt)
            % find all neighbors of a given point. Do not return anything
            % outside the bounds of the cellArray.
            points = [];
            num = 1;
            i = pt(1); j = pt(2);
            for ii= -1:1:1
                for jj= -1:1:1
                    if( ii == 0 && jj == 0); continue; end;
                    if(    i+ii >= 1 && i+ii <= size(obj.cellArray,1) ...
                       &&  j+jj >= 1 && j+jj <= size(obj.cellArray,2))
                        points(:,num) = [i+ii ; j+jj]; %#ok<AGROW>
                        num = num + 1;
                    end
                end
            end
        end
           
        function pt = xyToIj(obj,x,y)
            % convert x,y to i,j
            i = xToi(obj,x);
            j = yToj(obj,y);
            pt = [i ; j];
        end
        
        function pt = ijToXy(obj,i,j)
            % convert x,y to i,j
            x = iToX(obj,i);
            y = iToy(obj,j);
            pt = [x ; y];
        end
        
        function i = xToi(obj,x)
            % Convert x coordinate to row index.
            i = floor((x-obj.xMin)/obj.dx) + 1;  end  
        function j = yToj(obj,y)
            % Convert y coordinate to col index.
            j = floor((y-obj.yMin)/obj.dy) + 1;  end  
        function x = iToX(obj,i)
            % Convert row index to x coordinate
            x = obj.xMin + obj.dx*(i-0.5);  end % changed Aug 21, 2014, add xmin
        function y = jToY(obj,j)
            % convert col index to y coordinate
            y = obj.yMin + obj.dy*(j-0.5);  end % changed Aug 21, 2014 add ymin
        function dx = getDx(obj)
            % return x resolution
            dx = obj.dx; end;
        function dy = getDy(obj)
            % return x resolution
            dy = obj.dy; end;
    end
end