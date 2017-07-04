#include "SlamOmaticMap.h"


SlamOmaticMap::SlamOmaticMap(const unsigned int size, const uint8_t costMax, const uint8_t seuil, const double resMpPix)
  : IGeneric2dMap<uint8_t>(), m_first_update(true), m_seuil(seuil), m_maxcost(costMax), m_probability_map(new ProbabilityGridMap(size,resMpPix))
{
  //m_file = fopen("/home/vincent/Bureau/pose.csv","w");
  //m_cost_map = new CostMapV2(*m_probability_map,costMax,seuil);
  m_cost_map = new CostMap(*m_probability_map,costMax,seuil);
}

uint8_t SlamOmaticMap::at(const int x, const int y) const
{
  return m_probability_map->at(x,y);
}

uint8_t &SlamOmaticMap::at(const int x, const int y)
{
  return m_probability_map->at(x,y);
}

uint8_t SlamOmaticMap::at(const Pose2D &p) const
{
  return m_probability_map->at(p.x,p.y);
}

uint8_t &SlamOmaticMap::at(const Pose2D &p)
{
  return m_probability_map->at(p.x,p.y);
}

uint8_t SlamOmaticMap::at(const Point2D &p) const
{
  return m_probability_map->at(p.x,p.y);
}

uint8_t &SlamOmaticMap::at(const Point2D &p)
{
  return m_probability_map->at(p.x,p.y);
}

uint8_t SlamOmaticMap::costAt(const Pose2D &p) const
{
  return m_cost_map->at(p.x,p.y);
}

uint8_t SlamOmaticMap::costAt(const Point2D &p) const
{
  return m_cost_map->at(p.x,p.y);
}

uint8_t SlamOmaticMap::costMax() const
{
  return m_cost_map->max();
}

double SlamOmaticMap::resolution() const
{
  return m_probability_map->resolutionMpPix();
}

uint8_t SlamOmaticMap::costAt(const int x, const int y) const
{
  return m_cost_map->at(x,y);
}

void SlamOmaticMap::update(const Pose2D &p, const LiDAR_Scan_2D &scan)
{
  //m_mutex_prob.lock();
  unsigned int i, size = scan.size();

  //fprintf(m_file,"%lu\t%f\t%f\t%f\r\n",ros::Time::now(),p.x,p.y,p.theta);
  //fflush(m_file);

  LiDAR_Scan_2D loc_scan_pix(scan);
  int pix_x = p.x / resolution();
  int pix_y = p.y / resolution();

  // ___________________________
  // ::: MaJ Probability Map :::
  double _cos = cos(p.theta), _sin = sin(p.theta);
  double dist = 0.0f, seuil = SEUIL_DIST_CARTO*SEUIL_DIST_CARTO;

  //double cost = 0.0f;

  for(i = 0; i < size; i++)
  {
    if(scan.m_points[i].x == 0 && scan.m_points[i].y == 0) continue;

    dist = scan.m_points[i].x * scan.m_points[i].x + scan.m_points[i].y * scan.m_points[i].y;
    if(dist > seuil) {
      loc_scan_pix.m_points[i].x = 0;
      loc_scan_pix.m_points[i].y = 0;
    } else {
      loc_scan_pix.m_points[i].x = (int)((_cos*scan.m_points[i].x - _sin*scan.m_points[i].y + p.x) / resolution());
      loc_scan_pix.m_points[i].y = (int)((_sin*scan.m_points[i].x + _cos*scan.m_points[i].y + p.y) / resolution());
    }

  }

  std::vector<Point2D>::iterator it;
  it = std::unique (loc_scan_pix.m_points.begin(), loc_scan_pix.m_points.end());
  loc_scan_pix.m_points.resize( std::distance(loc_scan_pix.m_points.begin(),it) );

  //cout << "AVANT : " << scan.size() << " / APRES : " << loc_scan_pix.size() << endl;

  //cout << "Av1" << endl;
  //m_probability_map->razChange();
  //cout << "Ap1" << endl;
  for(i = 0; i < loc_scan_pix.size(); i++)
    if(loc_scan_pix.m_points[i].x != 0 && loc_scan_pix.m_points[i].y != 0)
      m_probability_map->pushLine(pix_x, pix_y, loc_scan_pix.m_points[i].x, loc_scan_pix.m_points[i].y,m_first_update);
        //at(loc_scan_pix.m_points[i].x,loc_scan_pix.m_points[i].y) == 50 );

  m_first_update = false;

  //m_mutex_prob.unlock();
  //m_mutex_cost.lock();

  // ____________________
  // ::: MaJ Cost Map :::

  delete m_cost_map;
  m_cost_map = new CostMap(*m_probability_map,m_maxcost,m_seuil);

  //m_mutex_cost.unlock();

}

nav_msgs::OccupancyGrid SlamOmaticMap::getCost2OccupencyGrid() const
{
  nav_msgs::OccupancyGrid cost;
  cost.header.stamp = ros::Time::now();
  cost.header.frame_id = "map";
  cost.info.height = m_cost_map->size();
  cost.info.width = m_cost_map->size();
  cost.info.resolution = m_cost_map->resolutionMpPix();
  cost.info.origin.position.x = -(m_cost_map->size()*m_cost_map->resolutionMpPix())/2.0;
  cost.info.origin.position.y = -(m_cost_map->size()*m_cost_map->resolutionMpPix())/2.0;
  cost.info.origin.position.z = 0;
  cost.data.resize(m_cost_map->globalsize());

  memcpy(cost.data.data(), m_cost_map->data(), m_cost_map->globalsize());

  return cost;
}

nav_msgs::OccupancyGrid SlamOmaticMap::getProb2OccupencyGrid() const
{
  nav_msgs::OccupancyGrid prob;
  prob.header.stamp = ros::Time::now();
  prob.header.frame_id = "map";
  prob.info.height = m_probability_map->size();
  prob.info.width = m_probability_map->size();
  prob.info.resolution = m_probability_map->resolutionMpPix();
  prob.info.origin.position.x = -(m_probability_map->size()*m_probability_map->resolutionMpPix())/2.0;
  prob.info.origin.position.y = -(m_probability_map->size()*m_probability_map->resolutionMpPix())/2.0;
  prob.info.origin.position.z = 0;
  prob.data.resize(m_probability_map->globalsize());

  memcpy(prob.data.data(), m_probability_map->data(), m_probability_map->globalsize());

  return prob;
}

bool SlamOmaticMap::save(const char *filename)
{
  //  cv::Mat prob, cost;
  //  std::string   str(filename), str_prob, str_cost;
  //  str_prob = str+"_prob.png";
  //  str_cost = str+"_cost.png";
  // Save ProbMap :
  //  prob = cv::Mat(m_probability_map->size(), m_probability_map->size(),CV_8UC1,m_probability_map->data(), m_probability_map->size()).clone();
  //  prob.convertTo(prob,CV_8UC1,2.55f);
  //  cv::imwrite(str_prob.c_str(),prob);
  // Save CostMap :
  //  cost = cv::Mat(m_cost_map->size(), m_cost_map->size(),CV_8UC1,m_cost_map->data(), m_cost_map->size()).clone();
  //  cost.convertTo(cost,CV_8UC1,255/m_cost_map->max());
  //  cv::imwrite(str_cost.c_str(),cost);
  return true;
}

bool SlamOmaticMap::load(const char *filename)
{
  //  cv::Mat prob, cost;
  //  int size;
  //  string str_prob, str_cost;
  //  str_prob = string(filename)+"_prob.png";
  //  str_cost = string(filename)+"_cost.png";
  //  //    ::: Load ProbMap :::
  //  try{
  ////    prob = cv::imread(str_prob.c_str(),cv::IMREAD_GRAYSCALE);
  ////    prob.convertTo(prob,CV_8UC1,1/2.55f);
  //    size = prob.total() * prob.elemSize();
  //    std::memcpy(m_probability_map->data(),prob.data,size);
  //  }catch(exception ex)
  //  {
  //    return false;
  //  }
  //  //    ::: Load CostMap :::
  //  try{
  ////    cost = cv::imread(str_cost.c_str(),cv::IMREAD_GRAYSCALE);
  ////    cost.convertTo(cost,CV_8UC1,1/2.55f);
  //    size = cost.total() * cost.elemSize();
  //    std::memcpy(m_cost_map->data(),cost.data,size);
  //  }catch(exception ex)
  //  {
  //    return false;
  //  }
  return true;
}

