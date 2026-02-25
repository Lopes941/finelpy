#pragma once    

#include <finelc/matrix.h>
#include <finelc/enumerations.h>

#include <finelc/mesh/node_iterators.h>

#include <memory>
#include <vector>
#include <string>
#include <numbers>
#include <cmath>
#include <math.h>

namespace finelc{

    /**
     * @brief Represents a point in space.
     * 
     * This class holds the data for a point in space,
     * the operations that can be done with points.
     * Multiplication here is defined as element-wise.
     */
    class Point
    {
        public:

            double x, y, z;


            /**
             * @brief Default constructor for Point.
             * 
             * This constructor initializes the point, putting 0 on all ungiven coorinates.
             * 
             * @param x x coorinate of point, default=0.
             * @param y y coorinate of point, default=0.
             * @param z z coorinate of point, default=0.
             */
            Point(double x=0, double y=0, double z=0): x(x), y(y), z(z) {}

            Point(const Vector& vec): x(vec(0)), y(vec(1)), z(vec(2)) {}

            Point(const Point&) =default;

            /**
             * @brief Addition operator between two points.
             * 
             * @param p2 The point to be added to the current one.
             */
            Point operator+(const Point& p2) const;

            /**
             * @brief Subtraction operator between two points.
             * 
             * @param p2 The point to be added to the current one.
             */
            Point operator-(const Point& p2) const;

            /**
             * @brief Multiplication by another point operator.
             * 
             * @param p2 The point to be multiplied.
             * 
             * This method performs element-wise multiplication between two points
             */
            Point operator*(const Point& p2) const;

            /**
             * @brief Multiplication by a scalar operator.
             * 
             * @param val The scalar to multiply by this point.
             */
            Point operator*(const double val) const;

            Point& operator+=(const Point& p2){
                x+=p2.x;
                y+=p2.y; 
                z+=p2.z;
                return *this;
            }

            Point& operator/=(const double& scalar){
                x/=scalar;
                y/=scalar; 
                z/=scalar;
                return *this;
            }

            bool operator== (const Point& point)const{
                return (x==point.x & y==point.y & z==point.z);
            }

            bool operator!= (const Point& point)const{
                return !(*this==point);
            }

            double dot(const Point& p)const{
                return x*p.x + y*p.y + z*p.z;
            }

            Vector as_vector()const{
                Vector vec_point(3);
                vec_point << x, y, z;
                return vec_point;
            }

            friend Point operator+(const Point& p, double val){
                return Point(p.x + val, p.y + val, p.z + val);
            }

            friend Point operator-(const Point& p, double val){
                return Point(p.x - val, p.y - val, p.z - val);
            }

    };

    extern double dist(const Point& p1, const Point& p2);
    extern double length(const Point& p);

    using Node = Point;
    using Point_ptr = std::shared_ptr<Point>;
    using Node_ptr = std::shared_ptr<Node>;
    using VectorNodes = std::vector<Node_ptr>;

    struct PlaneEquation{
        Vector n;
        double k;
    };

    inline Matrix points_to_matrix(const std::vector<Point>& points, int dimension=3){
        DenseMatrix out_mat(points.size(),dimension);

        for(int i=0; i<points.size(); i++){
            out_mat(i,0) = points[i].x;
            if(dimension >= 2){out_mat(i,1) = points[i].y;}
            if(dimension == 3){out_mat(i,2) = points[i].z;}
            
        }

        return out_mat;
    }

    /**
     * @brief Interface for a geometry.
     * 
     * This interface defines the basic structure for a geometry in the system.
     * It provides methods to retrieve the type of geometry. It should be given
     * information on the finite element nodes associated to it by the MeshBuilder
     */
    class IGeometry{

        private:
            NodeIterator_ptr iterator=nullptr;

        public:

            virtual ~IGeometry()=default;

            /**
             * @brief Get the name of the geometry.
             * 
             * This method should return the name of the geometry as a string.
             * It is expected that each derived class will implement this method
             * to return a unique name for the geometry.
             * 
             * @return const std::string& Name of the geometry.
             */
            virtual const GeometryType name()const=0;
            

            void set_iterator(NodeIterator_ptr& ite){
                iterator = std::move(ite);
            }


            virtual bool is_inside(const Point& p)const=0;

            int number_of_nodes()const{
                if(!iterator){
                    return 0;
                }
                return iterator->get_size();
            }

            std::vector<int> nodes(){

                if(number_of_nodes()==0){
                    return {};
                }

                std::vector<int> nds(number_of_nodes());

                int k = 0;
                iterator->reset();

                do{
                    nds[k++] = iterator->value();
                    ++(*iterator);
                }while(!iterator->done);

                return nds;
            }

    };

    using IGeometry_ptr = std::shared_ptr<IGeometry>;

    
    /**
     * @brief Represents a line in space.
     * 
     * This class holds the data for a line in space,
     * the operations that can be done with points.
     */
    class Line: public IGeometry{

        public:

            Point p1;
            Point p2;
            double theta;

            const GeometryType name() const override {return GeometryType::LINE;};

            /**
             * @brief Constructor of a line object, with given points.
             * 
             * @param p1 First point of the line
             * @param p2 Second point of the line
             */
            Line(Point p1_=Point(), Point p2_=Point()) :
                p1(p1_), 
                p2(p2_), 
                theta(std::atan2(p2_.y-p1_.y, p2_.x-p1_.x)) 
                {}

            Line(Point_ptr p1_, Point_ptr p2_) :
                p1(*p1_), 
                p2(*p2_), 
                theta(std::atan2(p2_->y-p1_->y, p2_->x-p1_->x))
                {}

            Line(const Line&) =default;
            ~Line()=default;

            double length()const;
            double length2()const;

            inline double distance(const Point& p)const{

                double l2 = length2();

                Vector u = (p-p1).as_vector();
                Vector w = (p2-p1).as_vector();
                double d = u.dot(w)/l2;

                if(d<=0){
                    return dist(p,p1);
                }else if(d>=1){
                    return dist(p,p2);
                }else{
                    Point m(p1.x+d*w(0), p1.y+d*w(1), p1.z+d*w(2));
                    return dist(p,m);
                }

            }

            inline bool is_inside(const Point& p)const override{
                double tolerance = 1e-7;
                return distance(p) <= tolerance;
            }

    };

    using Line_ptr = std::shared_ptr<Line>;

    /**
     * @brief Represents the result from the function get_line_intersection()
     * 
     * This class holds the data for a line in space,
     * the operations that can be done with points.
     */
    struct IntesectResult{
        bool is_intersect;
        Point intersection;
    };


    /**
     * @brief Interface for a 2D geometry (area).
     * 
     * This interface defines the basic structure for an area in the system.
     * It provides methods to retrieve the name of the area.
     */
    class IArea: public IGeometry{

        protected:
            std::shared_ptr<std::vector<Point>> vertices; /**< Vector with the vertices. */
            std::shared_ptr<std::vector<Line>> lines; /**< Vector with the vertices. */

        public:

            IArea()=default;

            IArea(const std::vector<Point>& ps):
                vertices(std::make_shared<std::vector<Point>>(ps)),
                lines(std::make_shared<std::vector<Line>>())
                {
                    for(int ipt=0; ipt<ps.size()-1;ipt++){
                        lines->push_back(Line(ps[ipt],ps[ipt+1]));
                    }
                    lines->push_back(Line(ps[ps.size()-1],ps[0]));
                }

            IArea(VectorNodes ps):
                lines(std::make_shared<std::vector<Line>>())
                {
                    vertices = std::make_shared<std::vector<Point>>();
                    vertices->reserve(ps.size());
                    for(auto& pt : ps){
                        vertices->emplace_back(*pt);
                    }
                    for(int ipt=0; ipt<ps.size()-1;ipt++){
                        lines->push_back(Line(ps[ipt],ps[ipt+1]));
                    }
                    lines->push_back(Line(ps[ps.size()-1],ps[0]));
                }

            /**
             * @brief Default destructor for IArea.
             * 
             * This destructor is defaulted and does not perform any additional actions.
             */
            ~IArea()=default;

            /**
             * @brief Get the name of the geometry.
             * 
             * This method should return the name of the geometry as a string.
             * It is expected that each derived class will implement this method
             * to return a unique name for the geometry.
             * 
             * @return const std::string& Name of the geometry.
             */
            const GeometryType name() const override {return GeometryType::POLYGON;};

            // /**
            //  * @brief Get the number of vertices from the geometry.
            //  * 
            //  * This method should return the number of vertices of the geometry.
            //  * 
            //  * @return size_t The number of vertices on the geometry.
            //  */
            inline size_t get_num_vertices() const {return vertices ? vertices->size() : 0;}

            // /**
            //  * @brief Get the vertices from the geometry.
            //  * 
            //  * This method should return the vertices of the geometry.
            //  * 
            //  * @return std::shared_ptr<std::vector<Point>> Vertices on the geometry.
            //  */
            inline std::shared_ptr<std::vector<Point>> get_vertices() const {return vertices ? vertices : nullptr;}

            // /**
            //  * @brief Get the number of lines from the geometry.
            //  * 
            //  * This method should return the number of lines of the geometry.
            //  * 
            //  * @return size_t The number of lines on the geometry.
            //  */
            inline size_t get_num_lines() const {return lines ? lines->size() : 0;}

            // /**
            //  * @brief Get the lines from the geometry.
            //  * 
            //  * This method should return the lines of the geometry.
            //  * 
            //  * @return std::shared_ptr<std::vector<Line>> Lines on the geometry.
            //  */
            inline std::shared_ptr<std::vector<Line>> get_lines() const {return lines ? lines : nullptr;}


            bool is_inside(const Point& p)const override;


            inline PlaneEquation get_equation()const{

                const Point& p1 = vertices->at(0);
                const Point& p2 = vertices->at(1);
                const Point& p3 = vertices->at(2);

                Eigen::Vector3d v1 = (p2-p1).as_vector();
                Eigen::Vector3d v2 = (p3-p1).as_vector();

                Vector n = v1.cross(v2);
                

                double k = p1.as_vector().dot(n);

                return PlaneEquation{n, k}; 

            }


            
    };

    using IArea_ptr = std::shared_ptr<IArea>;


    /**
     * @brief CRTP (Curiously Recurring Template Pattern) implementation of IGeometry.
     * 
     * This class implements the IGeometry interface using the CRTP pattern.
     * It allows derived classes to define their own methods while
     * providing a common interface for accessing the geometry features.
     * 
     * The Derived class must implement static_name() method to return the
     * appropriate value for the parent group.
     * 
     * @tparam Derived The derived class that implements the geometry.
     */
    template <typename Derived>
    class IAreaCRTP: public IArea{

        public:
        
            /**
             * @brief Default destructor for IAreaCRTP.
             * 
             * This destructor is defaulted and does not perform any additional actions.
             */
            ~IAreaCRTP()=default;

            
            /**
             * @brief Get the name of the geometry.
             * 
             * This method returns the name of the geometry by calling the static_name()
             * method of the Derived class. It is expected that each derived class will
             * implement this method to return a unique name.
             * 
             * @return const std::string& Name of the geometry.
             */
            inline const GeometryType name() const override{
                return Derived::static_name();
            }
            
    };
    
    class Rectangle: public IAreaCRTP<Rectangle>{

        private:

            /**
             * @brief Group name for the Rectangle geometry.
             * 
             * This static constant holds the name of the geometry, which is used to identify it in the system.
             */
            inline static const GeometryType name = GeometryType::RECTANGLE;

            Point dimension; /**< Dimensions of the Rectangle. Saved as Point datatype for easier operations. */
            Point origin; /**< Origin point of the Rectangle */


            void create_rectangle();
            

        public:

            /**
             * @brief Get the name of the geometry.
             * 
             * This static method returns the name of the geometry, which is used to identify it in the system.
             * 
             * @return const std::string& The name of the geometry.
             */
            static const GeometryType static_name() { return name;}


            Rectangle()=default;

            Rectangle(Point dimension, Point origin): dimension(dimension), origin(origin) {
                create_rectangle();
            }

            inline bool is_inside(const Point& p)const override{
                return p.x >= origin.x &&
                        p.x <= origin.x + dimension.x &&
                        p.y >= origin.y &&
                        p.y <= origin.y + dimension.y;
            }

            inline Point get_dimension() const {return dimension;}
            inline Point get_origin() const {return origin;}

            inline std::shared_ptr<Line> lower_side() const {
                return lines ? std::make_shared<Line>((*lines)[0]) : nullptr;}
                
            inline std::shared_ptr<Line> right_side() const {
                return lines ? std::make_shared<Line>((*lines)[1]) : nullptr;}

            inline std::shared_ptr<Line> upper_side() const {
                return lines ? std::make_shared<Line>((*lines)[2]) : nullptr;}

            inline std::shared_ptr<Line> left_side() const {
                return lines ? std::make_shared<Line>((*lines)[3]) : nullptr;}
        
    };
    
    using Rectangle_ptr = std::shared_ptr<Rectangle>;

    extern IntesectResult get_line_intersection(const Line& l1, const Line& l2, double tolerance=1e-7);
    extern IntesectResult get_line_area_intersection(IArea& l1, const Line& l2, double tolerance=1e-7);

    extern IArea& add_geometries(IArea&, IArea&);

    class IVolume: public IGeometry{

        protected:
            std::shared_ptr<std::vector<Point>> vertices; /**< Vector with the vertices. */
            std::shared_ptr<std::vector<Line>> lines; /**< Vector with the vertices. */
            std::shared_ptr<std::vector<IArea>> faces; /**< Vector with the faces. */

        public:

            IVolume()=default;

            IVolume(const std::vector<Point>& ps, const std::vector<IArea> fcs):
                vertices(std::make_shared<std::vector<Point>>(ps)),
                faces(std::make_shared<std::vector<IArea>>(fcs))
                {}

            /**
             * @brief Default destructor for IVolume.
             * 
             * This destructor is defaulted and does not perform any additional actions.
             */
            ~IVolume()=default;

            const GeometryType name() const override{return GeometryType::POLYHEDRON;};

            // /**
            //  * @brief Get the number of vertices from the geometry.
            //  * 
            //  * This method should return the number of vertices of the geometry.
            //  * 
            //  * @return size_t The number of vertices on the geometry.
            //  */
            inline size_t get_num_vertices() const {return vertices ? vertices->size() : 0;}

            // /**
            //  * @brief Get the vertices from the geometry.
            //  * 
            //  * This method should return the vertices of the geometry.
            //  * 
            //  * @return std::shared_ptr<std::vector<Point>> Vertices on the geometry.
            //  */
            inline std::shared_ptr<std::vector<Point>> get_vertices() const {return vertices ? vertices : nullptr;}

            // /**
            //  * @brief Get the number of lines from the geometry.
            //  * 
            //  * This method should return the number of lines of the geometry.
            //  * 
            //  * @return size_t The number of lines on the geometry.
            //  */
            inline size_t get_num_lines() const {return lines ? lines->size() : 0;}

            // /**
            //  * @brief Get the lines from the geometry.
            //  * 
            //  * This method should return the lines of the geometry.
            //  * 
            //  * @return std::shared_ptr<std::vector<Line>> Lines on the geometry.
            //  */
            inline std::shared_ptr<std::vector<Line>> get_lines() const {return lines ? lines : nullptr;}

            // /**
            //  * @brief Get the number of faces from the geometry.
            //  * 
            //  * This method should return the number of faces of the geometry.
            //  * 
            //  * @return size_t The number of faces on the geometry.
            //  */
            inline size_t get_num_faces() const {return faces ? faces->size() : 0;}

            // /**
            //  * @brief Get the faces from the geometry.
            //  * 
            //  * This method should return the faces of the geometry.
            //  * 
            //  * @return std::shared_ptr<std::vector<IArea>> Faces on the geometry.
            //  */
            inline std::shared_ptr<std::vector<IArea>> get_faces() const {return faces ? faces : nullptr;}


            bool is_inside(const Point& p)const override;
            
    };

    template <typename Derived>
    class IVolumeCRTP: public IVolume{

        public:
        
            /**
             * @brief Default destructor for IAreaCRTP.
             * 
             * This destructor is defaulted and does not perform any additional actions.
             */
            ~IVolumeCRTP()=default;

            
            /**
             * @brief Get the name of the geometry.
             * 
             * This method returns the name of the geometry by calling the static_name()
             * method of the Derived class. It is expected that each derived class will
             * implement this method to return a unique name.
             * 
             * @return const std::string& Name of the geometry.
             */
            inline const GeometryType name() const override{
                return Derived::static_name();
            }

            
    };

    class Hexahedron: public IVolumeCRTP<Hexahedron>{

        private:

            /**
             * @brief Group name for the Hexahedron geometry.
             * 
             * This static constant holds the name of the geometry, which is used to identify it in the system.
             */
            inline static const GeometryType name = GeometryType::HEXAHEDRON;

            Point dimension; /**< Dimensions of the Hexahedron. Saved as Point datatype for easier operations. */
            Point origin; /**< Origin point of the Hexahedron */


            void create_hex();
            

        public:

            /**
             * @brief Get the name of the geometry.
             * 
             * This static method returns the name of the geometry, which is used to identify it in the system.
             * 
             * @return const std::string& The name of the geometry.
             */
            static const GeometryType static_name() { return name;}


            Hexahedron()=default;

            Hexahedron(Point dimension, Point origin): dimension(dimension), origin(origin) {
                create_hex();
            }

            inline bool is_inside(const Point& p)const override{
                return p.x >= origin.x &&
                        p.x <= origin.x + dimension.x &&
                        p.y >= origin.y &&
                        p.y <= origin.y + dimension.y &&
                        p.z >= origin.z &&
                        p.z <= origin.z + dimension.z;
            }

            inline Point get_dimension() const {return dimension;}
            inline Point get_origin() const {return origin;}

            inline std::shared_ptr<IArea> lower_face() const {
                return faces ? std::make_shared<IArea>((*faces)[0]) : nullptr;}


            inline std::shared_ptr<IArea> front_face() const {
                return faces ? std::make_shared<IArea>((*faces)[1]) : nullptr;}
                
            inline std::shared_ptr<IArea> right_face() const {
                return faces ? std::make_shared<IArea>((*faces)[2]) : nullptr;}

            inline std::shared_ptr<IArea> back_face() const {
                return faces ? std::make_shared<IArea>((*faces)[3]) : nullptr;}

            inline std::shared_ptr<IArea> left_face() const {
                return faces ? std::make_shared<IArea>((*faces)[4]) : nullptr;}
        
            inline std::shared_ptr<IArea> upper_face() const {
                return faces ? std::make_shared<IArea>((*faces)[5]) : nullptr;}
    };

    using Hexahedron_ptr = std::shared_ptr<Hexahedron>;

} // namespace finelc
