#pragma once

#include <Eigen/Core>

#include <map>
#include <iostream>

namespace finelc{

    struct hdual{

        double val;
        std::map<int, double> eps;

        hdual(): val(0.){}
        hdual(double v): val(v){}
        hdual(int v): val(static_cast<double>(v)){}
        hdual(double v, std::map<int, double> e): val(v), eps(std::move(e)) {}

        hdual(const hdual&)=default;
        hdual(hdual&&)=default;

        hdual& operator=(const hdual&)=default;
        hdual& operator=(hdual&&)=default;

        int size()const{
            return std::prev(eps.end())->first +1;
        }

    };

    inline hdual operator+(const hdual& a, const hdual& b){
        hdual s = a;
        s.val += b.val;
        for(auto& [ind, v]:b.eps){
            s.eps[ind] += v;
        }
        return s;
    }

    inline hdual operator-(const hdual& a, const hdual& b){
        hdual s = a;
        s.val -= b.val;
        for(auto& [ind, v]:b.eps){
            s.eps[ind] -= v;
            if(s.eps[ind] == 0){
                s.eps.erase(ind);
            }
        }
        return s;
    }

    inline hdual operator*(const hdual& a, const hdual& b){
        hdual s(a.val*b.val,{});
        for(auto& [ind, v]: a.eps){
            s.eps[ind] = v*b.val;
        }
        for(auto& [ind, v]:b.eps){
            s.eps[ind] += v*a.val;
        }
        return s;
    }

    inline hdual operator/(const hdual& a, const hdual& b){
        hdual s(a.val/b.val,{});
        double b2 = b.val*b.val;
        double bv = b.val/b2;
        double av = a.val/b2;
        for(auto& [ind, v]: a.eps){
            s.eps[ind] = v*bv;
        }
        for(auto& [ind, v]:b.eps){
            s.eps[ind] -= v*av;
        }
        return s;
    }

    inline bool operator<(const hdual& a, const hdual& b){
        return std::abs(a.val) < std::abs(b.val);
    }

    inline bool operator>(const hdual& a, const hdual& b){ return b < a;}
    inline bool operator>=(const hdual& a, const hdual& b){ return !(a < b);}
    inline bool operator<=(const hdual& a, const hdual& b){ return !(b < a);}

    inline hdual& operator+=(hdual& a, const hdual& b){
        a.val += b.val;
        for(auto& [ind, v]:b.eps){
            a.eps[ind] += v;
        }
        return a;
    }

    inline hdual& operator-=(hdual& a, const hdual& b){
        a.val -= b.val;
        for(auto& [ind, v]:b.eps){
            a.eps[ind] -= v;
            if(a.eps[ind] == 0){
                a.eps.erase(ind);
            }
        }
        return a;
    }

    inline hdual& operator*=(hdual& a, const hdual& b){
        hdual s = a;
        a.val *= b.val;
        for(auto& [ind, v]: s.eps){
            a.eps[ind] = v*b.val;
        }
        for(auto& [ind, v]:b.eps){
            a.eps[ind] += v*s.val;
        }
        return a;
    }

    inline hdual& operator/=(hdual& a, const hdual& b){
        hdual s = a;
        a.val /= b.val;
        double b2 = b.val*b.val;
        double bv = b.val/b2;
        double sv = s.val/b2;
        for(auto& [ind, v]: s.eps){
            a.eps[ind] = v*bv;
        }
        for(auto& [ind, v]:b.eps){
            a.eps[ind] -= v*sv;
        }
        return a;
    }

    inline hdual operator-(const hdual& a){
        hdual r = a;
        r.val = -r.val;
        for(auto& [i,v] : r.eps){
            v = -v;
        }
        return r;
    }

    inline bool operator==(const hdual& a, const hdual& b){
        if(a.val != b.val) return false;
        if(a.eps.size() != b.eps.size()) return false;

        for(auto& [ind, v]: a.eps){
            auto it = b.eps.find(ind);
            if( it==b.eps.end() || it->second != v){
                return false;
            }
        }
        return true;
    }

    inline bool operator!=(const hdual& a, const hdual& b){
        return !(a == b);
    }

    inline bool isZero(const hdual& a){
        return a.val == 0.0 && a.eps.empty();
    }

    inline hdual abs(const hdual& a){
        if(a.val >= 0) return a;
        hdual r = -a;
        return r;
    }

    inline double abs2(const hdual& a){
        return a.val*a.val;
    }

    inline double norm(const hdual& a){
        return std::abs(a.val);
    }

    inline hdual sqrt(const hdual& a){
        hdual s(std::sqrt(a.val));
        for(auto& [ind, v]: a.eps){
            s.eps[ind] = v / (2. * s.val);
        }
        return s;
    }

    inline hdual conj(const hdual& a){return a;}
    inline hdual real(const hdual& a){return a;}
    inline hdual imag(const hdual&){return hdual();}

    inline std::ostream& operator<<(std::ostream& os, const hdual& a){
        os << a.val;
        return os;
    }
    
} // namespace finelc

namespace Eigen{

    template<>
    struct NumTraits<finelc::hdual> : NumTraits<double>{
        typedef finelc::hdual Real;
        typedef finelc::hdual NonInteger;
        typedef finelc::hdual Nested;

        enum{
            IsComplex = 0,
            IsInteger = 0,
            IsSigned = 1,
            RequireInitialization = 1,
            ReadCost = HugeCost,
            AddCost = HugeCost,
            MultCost = HugeCost,
        };


        static inline Real epsilon() {return finelc::hdual(std::numeric_limits<double>::epsilon());}
        static inline Real dummy_precision() {return epsilon();}
        static inline Real highest() {return finelc::hdual(std::numeric_limits<double>::max());}
        static inline Real lowest() {return finelc::hdual(std::numeric_limits<double>::lowest());}

        static inline bool isZero(const finelc::hdual& a){
            return a.val == 0. && a.eps.empty();
        }

    };
    

} // namespace Eigen


