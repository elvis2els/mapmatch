//auto-include {{{
#include <shapefil.h>
#include <boost/range/algorithm.hpp>
#include <boost/phoenix.hpp>
#include  <boost/property_tree/ptree.hpp>
#include  <boost/property_tree/ini_parser.hpp>
#include "ivmm/ivmm.h"
#include "gps.h"
#include "util.h"
//}}}
using namespace std;
namespace lambda = b::phoenix::arg_names;
namespace pt = b::property_tree;

static double oo = numeric_limits<double>::infinity();

inline static double normal(GpsPoint const& gps, Candidate const& cp, double mean, double stddev)
{
    static double sqrt_2_pi = sqrt( 2 * M_PI);
    double dist = bg::distance(gps, cp.point.geometry) - mean;
    double dist_sqr = dist * dist;
    double i = - dist_sqr / ( 2 * stddev * stddev);
    double c = 1.0 /  ( sqrt_2_pi * stddev);
    double n = c * exp(i);
    if ( n < 1e-8 ) n = 1e-8;
    return n;
}

double weight_speed(Path const& path, RoadMap const& map)
{
    struct SpeedGetter : public b::static_visitor<double>
    {
        RoadMap const& map;
        SpeedGetter(RoadMap const& map): map(map) {}

        double operator()(ARoadSegment const& ars)const
        {
            RoadSegment const& rs = map.roadsegment(ars.roadsegment_index);
            return rs.properties.get<double>("SPEED");
        }

        double operator()(PartOfRoad const& pr)const
        {
            RoadSegment const& rs = map.roadsegment(pr.roadsegment_index);
            return rs.properties.get<double>("SPEED");
        }

        double operator()(ProjectPoint const& )const
        {
            return 0;
        }
    };

    struct LengthGetter : public b::static_visitor<double>
    {
        double operator()(ARoadSegment const& ars)const
        {
            return ars.length;
        }

        double operator()(PartOfRoad const& pr)const
        {
            return pr.length;
        }

        double operator()(ProjectPoint const& )const
        {
            return 0;
        }
    };


    double d = path.total_length();
    double weightSpeed = 0;
    for(auto & e : path.entities)
    {
        weightSpeed += b::apply_visitor(LengthGetter(), e) / d * b::apply_visitor(SpeedGetter(map), e);
    }
    return weightSpeed;
}

static void initPre(
    IVMM::VVector<int> &pre,
    IVMM::VVector<Candidate> const& candidates, int begin, int end)
{
    int nCand = end - begin;
    pre.resize(nCand);
    for(int i = begin; i != end; ++i)
    {
        pre[i - begin].resize(candidates[i].size(), -1);
    }
}

IVMM::VVector<Candidate> IVMM::candidates(std::vector<GpsPoint> const &log) const
{
    VVector<Candidate> candidates;
    candidates.resize(log.size());
    for(size_t i = 0; i < log.size(); ++i)
    {
        double r = param.candidate_query_radious;
        vector<int> roadIdx = map_->query_road(log[i].geometry, r);     //查询候选路段id号
        while (roadIdx.empty())     //如果查询不到候选路段，逐渐增大查询范围
        {
            r *= 1.1;
            roadIdx = map_->query_road(log[i].geometry, r);
        }

        for(int ri : roadIdx)
        {
            RoadSegment const& rs = map_->roadsegment(ri);
            Candidate c;
            c.vote = 0;
            c.fvalue = 0.0;
            c.point = make_project_point(log[i].geometry, rs);//projectPoint(log[i], r.geometry);
            vector<Candidate> & can = candidates[i];
            if ( static_cast<int>(can.size()) < param.candidate_limit &&
                    ( can.empty() || !bg::equals(c.point.geometry, can.back().point.geometry)))
            {
                can.push_back(std::move(c));
            }
        }
    }
    return candidates;
}

template<typename V, typename C>
static void initVV(IVMM::VVector<V>& vv, IVMM::VVector<C> const& c)
{
    vv.resize(c.size());
    for(size_t i = 0; i < c.size(); ++i)
    {
        vv[i].resize(c[i].size());
    }
}

template<typename V, typename P>
static void initVVV(IVMM::VVVector<V>& vvv, IVMM::VVector<P> const& c)
{
    vvv.resize(c.size() - 1);       //代表的是每一个点到下一点的路径，所以最后一个点不需要
    for(size_t i = 1; i < c.size(); ++i)
    {
        vector<P> const& srcCand = c[i - 1];
        vector<P> const& destCand = c[i];
        vvv[i - 1].resize(srcCand.size());
        for(auto & e : vvv[i - 1])
        {
            e.resize(destCand.size());
        }
    }
}

template<typename V, typename P>
static void initVVV(IVMM::VVVector<V> & vvv, IVMM::VVVector<P> const& p)
{
    vvv.resize(p.size());
    for(size_t i = 0; i < p.size(); ++i)
    {
        vvv[i].resize(p[i].size());
        for(size_t j = 0; j < p[i].size(); ++j)
        {
            vvv[i][j].resize(p[i][j].size());
        }
    }
}

IVMM::VVector<double> IVMM::normal(std::vector<GpsPoint> const &log, IVMM::VVector<Candidate> const &candidates) const
{
    VVector<double> n;
    initVV(n, candidates);
    for(size_t i = 0; i < candidates.size(); ++i)
    {
        for(size_t j = 0; j < candidates[i].size(); ++j)
        {
            n[i][j] = ::normal(log[i], candidates[i][j], param.project_dist_mean, param.project_dist_stddev);
        }
    }

    return n;
}

IVMM::VVVector<Path> IVMM::paths(IVMM::VVector<Candidate> const &candidates) const
{
    VVVector<Path> paths;
    initVVV(paths, candidates);
    for(size_t i = 1; i < candidates.size(); ++i)
    {
        int srcGps = i - 1;
        int destGps = i;
        for(size_t srcCand = 0; srcCand < candidates[srcGps].size(); ++srcCand)         //求每个候选点之间的最短路径
        {
            for(size_t destCand = 0; destCand < candidates[destGps].size(); ++destCand)
            {
                paths[srcGps][srcCand][destCand] =
                    map_->shortest_path(candidates[srcGps][srcCand].point,
                                        candidates[destGps][destCand].point);
            }
        }
    }
    return paths;
}

double IVMM::find_sequence(
    vector<int> &seq,
    IVMM::VVector<double> const &n,
    IVMM::VVVector<Detail> const& vft,
    std::vector<GpsPoint> const &log,
    std::vector<double> const& w,
    IVMM::VVector<Candidate> const &candidates,
    int focusOnGps, int mustPassCand, int gpsBegin, int gpsEnd) const
{

    vector<double> f(candidates[gpsBegin].size());
    vector<double> initFocusf;    //用于保存focus点未经-oo化的值
    VVector<int> pre;
    initPre(pre, candidates, gpsBegin, gpsEnd);

    //init
    for(vector<int>::size_type i = 0; i < candidates[gpsBegin].size(); ++i)
    {
        f[i] = w[gpsBegin] * n[gpsBegin][i];
    }
    if ( gpsBegin == focusOnGps)        //将除了focus的第mustPass个候选点的值改为-oo
    {
        initFocusf = f;
        for(vector<int>::size_type i = 0; i < candidates[focusOnGps].size(); ++i)
        {
            if ( i != (unsigned int)mustPassCand )
            {
                f[i] = -oo;
            }
        }
    }

#ifdef QT_QML_DEBUG
    cout<<"FocusOnGps: "<<focusOnGps<<endl;
    for(auto n: f)
        cout<< n<< " ";
    cout<<endl;
#endif

    for(int destGps = gpsBegin + 1; destGps < gpsEnd; ++destGps)
    {
        vector<double> newF(candidates[destGps].size());
        int srcGps = destGps - 1;
        bool hasTurePath = false;       //标记是否有候选点可以在合理速度下到达下一gps候选点
        for(vector<int>::size_type destCand = 0; destCand < candidates[destGps].size(); ++destCand)
        {
            //find max pre candidate for destCand
            double maxF = -oo;
            int preCandIdx = -1;          
            for(vector<double>::size_type srcCand = 0; srcCand < f.size(); ++srcCand)
            {                
                Detail const detail = vft[srcGps][srcCand][destCand];

#ifdef QT_QML_DEBUG
                cout<<"srcGps: "<<srcGps<<"srcCand: "<<srcCand<<" destGps: "<<destGps<<" destCand:"<<destCand<<endl;
                cout<<"avg speed: "<<detail.avg_speed<<" weight speedf: "<<detail.weight_speed*param.factor<<endl;
#endif

                if(detail.avg_speed > detail.weight_speed * param.factor)        //忽略真实平均速度速度超过道路加权限速合理范围的点
                    continue;
//                if(detail.avg_speed == 0 && !bg::equals(candidates[srcGps][srcCand].point,candidates[destGps][destCand].point))
//                    continue;

                //前一点是可达的点才认为是存在可以到达下一点的点
                if(srcGps == focusOnGps)
                {
                    if(initFocusf[srcCand] != -oo)
                        hasTurePath = true;
                }
                if(f[srcCand] == -oo)   //忽略不可达的候选点
                    continue;
                hasTurePath = true;

                //获取src和des两候选点所在路段的限速
                RoadSegment const& srcRs = map_->roadsegment(candidates[srcGps][srcCand].point.index);
                RoadSegment const& desRs = map_->roadsegment(candidates[destGps][destCand].point.index);
                double srcRsSpeed = srcRs.properties.get<double>("SPEED");
                double desRsSpeed = desRs.properties.get<double>("SPEED");
                double rsSpeedFac = srcRsSpeed / (abs(srcRsSpeed - desRsSpeed) + srcRsSpeed);

                double fst = n[destGps][destCand] *
                             detail.v * detail.ft * rsSpeedFac;

#ifdef QT_QML_DEBUG
                cout<<"n: "<<n[destGps][destCand]<<" v: "<<detail.v<<" ft: "<<detail.ft<<" speedF: "<<rsSpeedFac<<endl;
#endif

                if(detail.avg_speed != 0 && !(candidates[destGps].size() == 1 && f.size() == 1))   //同一个点和两点之间只有唯一路径的就不要考虑了
                    if(fst < 5.0e-05)   //忽略边权值太小的候选点
                    {
#ifdef QT_QML_DEBUG
                        cout << "fst is too small: " << fst <<endl;
#endif
                        continue;
                    }

                fst *= w[srcGps];

#ifdef QT_QML_DEBUG
                cout<<"fst: "<<fst<<endl;
#endif

                double sum;
                sum = f[srcCand] + fst;

#ifdef QT_QML_DEBUG
                cout <<"maxF:"<<maxF<<" sum:"<<sum<<endl;
#endif

                if (sum > maxF)
                {
                    maxF = sum;
                    preCandIdx = srcCand;
                }
            }
            newF[destCand] = maxF;
            pre[destGps - gpsBegin][destCand] = preCandIdx;
        }

        if (b::all(newF, lambda::arg1 == -oo) )
        {
            if(hasTurePath)
                return -1;
            //由于地图原因导致达不可达点，仅做空间分析
#ifdef QT_QML_DEBUG
            cout<<"!hasTurePath"<<endl;
#endif
            for(vector<int>::size_type destCand = 0; destCand < candidates[destGps].size(); ++destCand)
            {
                double maxF = -oo;
                int preCandIdx = -1;
                for(vector<double>::size_type srcCand = 0; srcCand < f.size(); ++srcCand)
                {
                    Detail const detail = vft[srcGps][srcCand][destCand];
                    if(f[srcCand] == -oo)   //忽略不可达的候选点
                        continue;

                    double fst = w[srcGps] *
                                 n[destGps][destCand] *
                                 detail.v;

                    double sum;
                    sum = f[srcCand] + fst;

#ifdef QT_QML_DEBUG
                    cout<<"srcGps: "<<srcGps<<"srcCand: "<<srcCand<<" destGps: "<<destGps<<" destCand:"<<destCand<<endl;
                    cout<<"avgSpeed: "<<detail.avg_speed<<" fst: "<<fst<<" maxF:"<<maxF<<" sum:"<<sum<<endl;
#endif

                    if (sum > maxF)
                    {
                        maxF = sum;
                        preCandIdx = srcCand;
                    }
                }
                newF[destCand] = maxF;
                pre[destGps - gpsBegin][destCand] = preCandIdx;
            }
        }

        if ( destGps == focusOnGps )
        {
            initFocusf = newF;
            for(vector<double>::size_type i = 0; i < newF.size(); ++i)
            {
                if ( i != (unsigned int)mustPassCand )
                {
                    newF[i] = -oo;
                }
            }
        }
        if (b::all(newF, lambda::arg1 == -oo) )
        {
                return -1;
        }
        f = std::move(newF);        //新的f值数组替换掉前一个点的，省空间

#ifdef QT_QML_DEBUG
        cout<<"desGps: "<<destGps<<endl;
        for(auto n: f)
            cout<< n<< " ";
        cout<<endl<<endl;
#endif

    }

    int idx = max_element(f.begin(), f.end()) - f.begin();
    double fvalue = f[idx];
    int nGps = gpsEnd - gpsBegin;
    seq.resize(nGps);

    while(nGps--)
    {
#ifdef QT_QML_DEBUG
        cout<<idx<<"<-";
#endif

        seq[nGps] = idx;//const_cast<Candidate *>(&candidates[nGps][idx]);
        idx = pre[nGps][idx];
    }

#ifdef QT_QML_DEBUG
    cout<<"fvalue: "<<fvalue<<endl;
    cout<<endl<<endl;
#endif

    return fvalue;
}

static pair<int, int> window(int i, int w, int sz)
{
    int begin, end;
    if ( w <= 0)
    {
        begin = 0;
        end = sz;
    }
    else
    {
        begin = max(0, i - w);
        end = min(i + w, sz);
    }
    return { begin, end };
}

bool IVMM::map_match(vector<GpsPoint> const &log,
                     VVector<double> & n,
                     VVVector<Detail> &details,
                     VVVector<Path>& paths,
                     VVector<Candidate>& candidates,
                     std::vector<int>& finalCand)const
{
    candidates = this->candidates(log);
    paths = this->paths(candidates);
    n = normal(log, candidates);        //计算正态分布值
    details = this->detail(log, paths);

    /* debug：用于查看gps点的候选点位置  */
#ifdef QT_QML_DEBUG
    for(size_t i=0; i<log.size(); ++i)
    {
        cout <<  "GPS: "<< i << "   " <<  setiosflags(ios::fixed) << setprecision(16) << log[i].geometry.x() << "," << log[i].geometry.y() << endl;
        for(size_t j=0; j<candidates[i].size() ; ++j)
        {
            cout << j << ": " << candidates[i][j].point.geometry.x() << "," << candidates[i][j].point.geometry.y() << endl;
        }
        cout<<endl;
    }
#endif

    vector<double> w(log.size() - 1, 1.0);      //和其余n-1个点的权值

    int sz = (int) log.size();
    for(vector<GpsPoint>::size_type i = 0; i < log.size(); ++i)
    {
        int begin, end;
        tie(begin, end) = window(i, param.window, sz);
        //here weight for gps i
        initW(w, log, begin, end, i);
        vector<Candidate>::size_type badConnection = 0;
        for(vector<Candidate>::size_type k = 0; k < candidates[i].size(); ++k)
        {
            vector<int> sequence;
            double fvalue = find_sequence(sequence, n, details, log, w , candidates, i, k, begin, end);     //得到文中fvalue和经过候选点k的最佳路径
            if ( fvalue < 0)
            {
                ++badConnection;
            }
            else
            {
                for(vector<int>::size_type b = 0; b < sequence.size(); ++b)
                {
                    ++candidates[b + begin][sequence[b]].vote;
                }
            }
            candidates[i][k].fvalue = fvalue;
        }
        if ( badConnection == candidates[i].size() )
        {
#ifdef QT_QML_DEBUG
            cout << "gps " << i << " match fail" << endl;
#endif
            return false;
        }
    }

    finalCand.resize(candidates.size());
    for(VVector<Candidate>::size_type i = 0; i < candidates.size(); ++i)
    {
//        finalCand[i] = b::max_element<b::return_begin_found>(candidates[i], [](Candidate const & a, Candidate const & b){return make_pair(a.vote, a.fvalue) < make_pair( b.vote, b.fvalue);}).size();
        finalCand[i] = max_element(candidates[i].begin(), candidates[i].end(), [](Candidate const & a, Candidate const & b){return make_pair(a.vote, a.fvalue) < make_pair( b.vote, b.fvalue);})  - candidates[i].begin();
    }

#ifdef QT_QML_DEBUG
    cout<<"finalCand: ";
    for(auto n: finalCand)
        cout<<n<<"->";
    cout << endl;
#endif

    return true;
}

IVMM::VVVector<Detail> IVMM::detail(std::vector<GpsPoint> const &log, IVMM::VVVector<Path> const &paths) const
{
    VVVector<Detail> details;
    initVVV(details, paths);
    for(VVVector<Path>::size_type srcGps = 0; srcGps < paths.size(); ++srcGps)
    {
        int destGps = srcGps + 1;
        double distanceOfTowGps = bg::distance(log[srcGps], log[destGps]);
        int timeInterval = (log[destGps].time - log[srcGps].time).total_seconds();
        for(VVector<Path>::size_type srcCand = 0; srcCand < paths[srcGps].size(); ++srcCand)
        {
            for(vector<Path>::size_type destCand = 0; destCand < paths[srcGps][srcCand].size(); ++destCand)
            {
                Detail & detail = details[srcGps][srcCand][destCand];
                Path const& path = paths[srcGps][srcCand][destCand];
                double pathLength = path.total_length();
                double d = distanceOfTowGps + 1;
                double l = pathLength + 1;
                detail.v = path.valid() ? min((d / l) * (d / l), sqrt(l / d)) : -oo;        //这个位置被修改过

                double weightSpeed = pathLength == 0 ? 0 : ::weight_speed(path, *map_);
                double avgSpeed = pathLength / timeInterval;

                if  ( weightSpeed == 0 )
                {
                    detail.ft = 1;
                }
                else
                {
                    detail.ft = weightSpeed / (abs(avgSpeed - weightSpeed) + weightSpeed);
                }

                //=============
                detail.weight_speed = weightSpeed;
                detail.avg_speed = avgSpeed;
                detail.path_length = pathLength;
                detail.time_inteval = timeInterval;
                detail.two_gps_distance = distanceOfTowGps;
            }
        }
    }
    return details;
}

void IVMM::initW(std::vector<double> &w, std::vector<GpsPoint> const &log, int begin, int end, int focusOnGps) const
{
    for(int m = begin; m < end - 1; ++m)
    {
        int otr = m;
        if ( focusOnGps <= m ) otr++;
        double d = bg::distance(log[focusOnGps], log[otr]);
        w[m] = exp(-d * d / (param.beta * param.beta));
    }
}

bool IVMM::st_match(
    std::vector<GpsPoint> const &log,
    IVMM::VVector<double> &n,
    IVMM::VVVector<Detail> &details,
    IVMM::VVVector<Path> &paths,
    IVMM::VVector<Candidate> &candidates,
    std::vector<int> &finalCand) const
{

    candidates = this->candidates(log);
    paths = this->paths(candidates);
    n = normal(log, candidates);
    details = this->detail(log, paths);
    vector<double> w(log.size() - 1, 1.0);
    if (find_sequence(finalCand, n, details, log, w, candidates, -1, -1, 0, log.size()) < 0)
    {
        return false;
    }
    return true;
}

vector<Path> IVMM::map_match(std::vector<GpsPoint> const& log)const
{
    vector<Path> finalPath;
    VVector<double> n;
    VVVector<Detail> details;
    VVVector<Path> paths;
    VVector<Candidate> candidates;
    vector<int> finalCand;
    if ( ! map_match(log, n, details, paths, candidates, finalCand) )
    {
        return finalPath;
    }
    for(VVVector<Path>::size_type i = 0; i < paths.size(); ++i)
    {
        int srcBest = finalCand[i];
        int destBest = finalCand[i + 1];
        finalPath.push_back(std::move(paths[i][srcBest][destBest]));
    }
    return finalPath;
}

void IVMM::draw_paths_to_shp(
    char const* output,
    vector<int> const& finalCand,
    IVMM::VVVector<Path> const& paths,
    IVMM::VVVector<Detail> const& details,
    IVMM::VVector<double> const& n)const
{
    RoadMap const& bjRoad = *map_;
    SHPHandle shp = SHPCreate(output, SHPT_ARC);
    DBFHandle dbf = DBFCreate(output);
    DBFAddField(dbf, "ID", FTInteger, 10, 0);
    DBFAddField(dbf, "FROMGPS", FTInteger, 10, 0);
    DBFAddField(dbf, "FROMCAND", FTInteger, 10, 0);
    DBFAddField(dbf, "TOGPS", FTInteger, 10, 0);
    DBFAddField(dbf, "TOCAND", FTInteger, 10, 0);
    DBFAddField(dbf, "LENGTH", FTDouble, 10, 3);
    DBFAddField(dbf, "GPS_DISTANCE", FTDouble, 10, 3);
    DBFAddField(dbf, "V", FTDouble, 12, 6);
    DBFAddField(dbf, "GPS_TIME_INTERVAL", FTInteger, 10, 0);
    DBFAddField(dbf, "AVERAGE_SPEED", FTDouble, 12, 6);
    DBFAddField(dbf, "WEIGHT_SPEED", FTDouble, 12, 6);
    DBFAddField(dbf, "FT", FTDouble, 12, 6);
    DBFAddField(dbf, "FSFT", FTDouble, 12, 6);
    DBFAddField(dbf, "N", FTDouble, 12, 6);

    int ID = 0;
    for(int  g = 0 ; g < paths.size(); ++g)
    {
        for( int  c1 = 0; c1 < paths[g].size(); ++c1)
        {
            for(int c2 = 0; c2 < paths[g][c1].size(); ++c2)
            {
                if ( c1 != finalCand[g] || c2 != finalCand[g + 1])
                    continue;
                Detail const& detail = details[g][c1][c2];
                Path const& path = paths[g][c1][c2];
                Linestring line = geometry(path, bjRoad);
                double x[line.size()];
                double y[line.size()];
                b::transform(line, x, [](Point const & p)
                {
                    return p.x();
                });
                b::transform(line, y, [](Point const & p)
                {
                    return p.y();
                });
                SHPObject* l = SHPCreateSimpleObject(SHPT_ARC, line.size() , x, y, nullptr);
                int id = SHPWriteObject(shp, -1, l);
                DBFWriteIntegerAttribute(dbf, id, 0, ID++);
                DBFWriteIntegerAttribute(dbf, id, 1, g);
                DBFWriteIntegerAttribute(dbf, id, 2, c1);
                DBFWriteIntegerAttribute(dbf, id, 3, g + 1);
                DBFWriteIntegerAttribute(dbf, id, 4, c2);
                DBFWriteDoubleAttribute(dbf, id, 5, detail.path_length);
                DBFWriteDoubleAttribute(dbf, id, 6, detail.two_gps_distance);
                DBFWriteDoubleAttribute(dbf, id, 7, detail.v);
                DBFWriteIntegerAttribute(dbf, id, 8, detail.time_inteval);
                DBFWriteDoubleAttribute(dbf, id, 9, detail.avg_speed);
                DBFWriteDoubleAttribute(dbf, id, 10, detail.weight_speed);
                DBFWriteDoubleAttribute(dbf, id, 11, detail.ft);
                DBFWriteDoubleAttribute(dbf, id, 12, detail.v * detail.ft * n[g + 1][c2]);
                DBFWriteDoubleAttribute(dbf, id, 13, n[g + 1][c2]);
                SHPDestroyObject(l);
            }
        }
    }
    DBFClose(dbf);
    SHPClose(shp);
}

vector<Path> IVMM::map_match_s(vector<GpsPoint> const& log, vector<pair<int, int> >& ranges)const
{
    vector<Path> paths = map_match(log);
    if ( paths.empty() )
    {
        return paths;
    }
    //找到path中有效的部分放到ranges中
    auto pathBegin = find_if(paths.begin(), paths.end(), [](Path const & p) { return p.valid(); });
    auto gpsBegin = log.begin() + distance(paths.begin(), pathBegin);
    while( pathBegin < paths.end() )
    {
        auto firstInvalidPath = find_if(pathBegin, paths.end(), [](Path const & p) { return !p.valid(); });
        auto invalidPathBeginGps = gpsBegin + distance(pathBegin, firstInvalidPath);
        ranges.emplace_back(distance(log.begin(), gpsBegin), distance(log.begin(), invalidPathBeginGps + 1));
        pathBegin = find_if(firstInvalidPath, paths.end(), [](Path const & p) { return p.valid(); });
        gpsBegin = log.begin() + distance(paths.begin(), pathBegin);
    }
    return paths;
}

boost::optional<IVMMParam> IVMMParam::load_config(std::string const& filename)
{
    pt::ptree pt;
    IVMMParam param;
    try
    {
        pt::read_ini(filename, pt);
        param.project_dist_mean = pt.get<double>("IVMM.projectDistMean");
        param.project_dist_stddev = pt.get<double>("IVMM.projectDistStddev");
        param.candidate_query_radious = pt.get<double>("IVMM.candidateQueryRadious");
        param.candidate_limit = pt.get<int>("IVMM.candidateLimit");
        param.beta = pt.get<double>("IVMM.beta");
        param.window = pt.get<int>("IVMM.window");
    }
    catch(std::exception const& e)
    {
        std::cerr << e.what() << endl;
        return boost::none;
    }
    return param;
}
