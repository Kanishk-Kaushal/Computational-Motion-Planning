% Collision Detecting Function using Seperating Axis Theorem (SAT)
function Flag = triangle_intersection(P1, P2)

% Test Case
% P1 = [1, 2, 1; 1, 2, 3 ]';
% P2 = [1.5, 3, 4; 2.5, 4, 3]';

% triangle_test : returns true if the triangles overlap and false otherwise
%%% All of your code should be between the two lines of stars.

% *******************************************************************'

% Flags
Flag_1 = false;
Flag_2 = false;

% For a Triangle ABC
A = P1(1, :);
B = P1(2, :);
C = P1(3, :);

% Checking P2 in triangle P1
for i = 1:length(P2)
    P = P2(i, :);
    if(Is_Point_in_Triangle(P, A, B, C))
        Flag_1  = true;
        break;
    end
end

% For a Triangle ABC
A = P2(1, :); 
B = P2(2, :); 
C = P2(3, :);
 
% Checking P1 in triangle P2
for i =1:length(P1)
    P = P1(i, :);
    if(Is_Point_in_Triangle(P, A, B, C))
        Flag_2  = true;
        break;
    end
end

Flag = Flag_1 || Flag_2;

% *******************************************************************
end

function Point_Status = Is_Point_in_Triangle(Point,a,b,c)

    % Point Status  => True => Point Present in Triangle & False => Point
    % Outside triangle

    % This Problem is solved using Convex Hull's Principles
    % Refer : https://mathworld.wolfram.com/TriangleInterior.html

    AC_Vector = c - a;
    BA_Vector = b - a;
    PA_Vector = Point - a;

    Dot_AC_AC = dot(AC_Vector, AC_Vector);
    Dot_AC_BA = dot(AC_Vector, BA_Vector);
    Dot_AC_PA = dot(AC_Vector, PA_Vector);
    Dot_BA_BA = dot(BA_Vector, BA_Vector);
    Dot_BA_PA = dot(BA_Vector, PA_Vector);

   Inverse_Denominator = 1/(Dot_AC_AC*Dot_BA_BA - Dot_AC_BA*Dot_AC_BA);
    U = (Dot_BA_BA*Dot_AC_PA - Dot_AC_BA*Dot_BA_PA)*Inverse_Denominator;
    V = (Dot_AC_AC*Dot_BA_PA - Dot_AC_BA*Dot_AC_PA)*Inverse_Denominator;

    Point_Status = ((U >= 0) && (V >= 0) && (U + V) <= 1);

    % Another and "Easier-To-Understand BUT Harder-To-Computer" Method would be if a Point P lies 
    % inside a Triangle ABC then Area(ABC) = Area(PAB) + Area(PBC) + Area(PCA)
    % The Area could be calculated using Heron's Formula.

end
