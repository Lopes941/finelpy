#include <finelc/geometry/geometry.h>

#include <iostream>
#include <cmath>

namespace finelc{

    std::vector<Point> get_points(
        const std::vector<Point> & pts, 
        const std::vector<int>& inds){

            std::vector<Point> ind_pts;
            ind_pts.reserve(inds.size());

            for(auto& ind:inds){
                ind_pts.emplace_back(pts.at(ind));
            }
            return ind_pts;
        }

    Point Point::operator+(const Point& p2) const{
        return Point(x+p2.x, y+p2.y, z+p2.z);
    }

    Point Point::operator-(const Point& p2) const{
        return Point(x-p2.x, y-p2.y, z-p2.z);
    }

    Point Point::operator*(const Point& p2) const{
        return Point(x*p2.x, y*p2.y, z*p2.z);
    }

    Point Point::operator*(const double val) const{
        return Point(x*val, y*val, z*val);
    }

    double dist2(const Point& p1, const Point& p2){
        Point diff = p2-p1;
        return diff.x*diff.x + diff.y*diff.y + diff.z*diff.z;
    }

    double dist(const Point& p1, const Point& p2){
        return std::sqrt(dist2(p1,p2));
    }

    double length(const Point& p){
        return std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
    }

    double Line::length()const{
        return dist(p1,p2);
    }

    double Line::length2()const{
        return dist2(p1,p2);
    }
    

    bool IArea::is_inside(const Point& p)const{

        auto [n, k] = get_equation();

        // Must lie on plane
        if (std::abs(n.dot(p.as_vector()) - k) > 1e-7)
            return false;

        for (size_t i = 0; i < vertices->size(); ++i)
        {
            const Point& a = (*vertices)[i];
            const Point& b = (*vertices)[(i+1) % vertices->size()];

            Eigen::Vector3d edge = b.as_vector() - a.as_vector();
            Eigen::Vector3d toP  = p.as_vector() - a.as_vector();

            Vector cross = edge.cross(toP);

            if (n.dot(cross) < - 1e-7)
                return false;
        }

        return true;

    }
    
    void Rectangle::create_rectangle(){

        vertices = std::make_unique<std::vector<Point>>(4);
        lines = std::make_unique<std::vector<Line>>(4);
        auto& vert = *vertices;
        auto& line = *lines;

        vert[0] = origin;
        vert[1] = origin + Point(dimension.x,0,0);
        vert[2] = origin + Point(dimension.x,dimension.y,0);
        vert[3] = origin + Point(0,dimension.y,0);

        line[0] = Line(vert[0],vert[1]);
        line[1] = Line(vert[1],vert[2]);
        line[2] = Line(vert[2],vert[3]);
        line[3] = Line(vert[3],vert[0]);
    }

    void Hexahedron::create_hex(){

        vertices = std::make_unique<std::vector<Point>>(8);
        lines = std::make_unique<std::vector<Line>>(12);
        faces = std::make_unique<std::vector<IArea>>(6);

        auto& vert = *vertices;
        auto& line = *lines;
        auto& face = *faces;

        vert[0] = origin;
        vert[1] = origin + Point(dimension.x,0,0);
        vert[2] = origin + Point(dimension.x,dimension.y,0);
        vert[3] = origin + Point(0,dimension.y,0);
        vert[4] = origin + Point(0,0,dimension.z);
        vert[5] = origin + Point(dimension.x,0,dimension.z);
        vert[6] = origin + Point(dimension.x,dimension.y,dimension.z);
        vert[7] = origin + Point(0,dimension.y,dimension.z);

        line[0] = Line(vert[0],vert[1]);
        line[1] = Line(vert[1],vert[2]);
        line[2] = Line(vert[2],vert[3]);
        line[3] = Line(vert[3],vert[0]);

        line[4] = Line(vert[0],vert[4]);
        line[5] = Line(vert[1],vert[5]);
        line[6] = Line(vert[2],vert[6]);
        line[7] = Line(vert[3],vert[7]);

        line[8] =  Line(vert[4],vert[5]);
        line[9] =  Line(vert[5],vert[6]);
        line[10] = Line(vert[6],vert[7]);
        line[11] = Line(vert[7],vert[4]);

        face[0] = IArea(get_points(vert,{0,3,2,1}));

        face[1] = IArea(get_points(vert,{0,1,5,4}));
        face[2] = IArea(get_points(vert,{1,2,6,5}));
        face[3] = IArea(get_points(vert,{2,3,7,6}));
        face[4] = IArea(get_points(vert,{3,0,4,7}));

        face[5] = IArea(get_points(vert,{4,5,6,7}));
    }

    IntesectResult get_line_intersection(const Line& l1, const Line& l2, double tolerance){

        Point p = l1.p1;
        Point q = l2.p1;

        Point r = l1.p2 - l1.p1;
        Point s = l2.p2 - l2.p1;

        Point qp = q - p;

        double r_dot_r = r.dot(r);
        double s_dot_s = s.dot(s);
        double r_dot_s = r.dot(s);
        double r_dot_qp = r.dot(qp);
        double s_dot_qp = s.dot(qp);

        double denom = r_dot_r * s_dot_s - r_dot_s * r_dot_s;

        if (std::fabs(denom) < tolerance)
            return {false, Point(0,0,0)}; // Parallel or nearly parallel

        double t = (r_dot_qp * s_dot_s - s_dot_qp * r_dot_s) / denom;
        double u = (r_dot_qp + t * r_dot_s) / s_dot_s;

        if (t < 0 || t > 1 || u < 0 || u > 1)
            return {false, Point(0,0,0)};

        Point intersection1 = p + r * t;
        Point intersection2 = q + s * u;

        if (length(intersection1 - intersection2) > tolerance)
            return {false, Point(0,0,0)}; // skew lines

        return {true, intersection1};
    }

    IntesectResult get_line_area_intersection(IArea& A, const Line& l, double tolerance){

        auto [n,k] = A.get_equation();
        Vector p1 = l.p1.as_vector();
        Vector dp = l.p2.as_vector() - p1;

        double den = n.dot(dp);

        // Parallel case
        if (std::abs(den) < tolerance)
        {
            if (std::abs(k - n.dot(p1)) < tolerance){
                // Coplanar
                return {true, l.p1};
            }
            // No intersection
            return {false, Point{}};
        }

        double t = (k - n.dot(p1)) / den;

        if (t < -tolerance || t > 1.0 + tolerance)
            return {false, Point{}};

        Point p = l.p1 + (l.p2 - l.p1) * t;

        return {true, p};

    }

    IArea& add_geometries(IArea& geo1, IArea& geo2){
        return geo1;
    }
    

    bool IVolume::is_inside(const Point& p)const{

        double minimum_x = p.x;
        for(auto& point: *vertices){
            minimum_x = minimum_x>point.x ? point.x : minimum_x;
        }
        minimum_x -= 1.;

        int num_intersect = 0;

        Line x_line = Line(Point(minimum_x,p.y,p.z), p);
        for(auto& face : *faces){
            if(face.is_inside(p)) return true;
            if(get_line_area_intersection(face, x_line).is_intersect){
                num_intersect++;
            }
        }
        return num_intersect % 2 == 1;

    }

} // namespace finel