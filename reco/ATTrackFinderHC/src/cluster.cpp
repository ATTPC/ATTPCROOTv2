#include "cluster.h"

#include <fstream>
#include <limits>
#include <cctype>

#include <boost/algorithm/string/replace.hpp>

void Cluster::calculateRelationshipMatrixIfNecessary() const
{
  if (this->relationshipMatrix.size() == 0)
    {
      this->relationshipMatrix = Eigen::ArrayXXi(this->pointIndexCount, this->pointIndexCount);
      this->relationshipMatrix.fill(0);

      //for (pcl::PointIndicesConstPtr const &cluster : this->getClusters())
      for (size_t iii=0; iii < this->clusters.size(); iii++)
        {
          pcl::PointIndicesConstPtr const cluster = this->clusters[iii];
          std::vector<int> const &indices = cluster->indices;
          size_t indicesSize = indices.size();

          for (size_t i = 0; i < indicesSize; ++i)
            {
              int indexI = indices[i];

              ++this->relationshipMatrix(indexI, indexI);

              for (size_t j = i + 1; j < indicesSize; ++j)
                {
                  int indexJ = indices[j];

                  if(indexI < indexJ)
                    ++this->relationshipMatrix(indexI, indexJ);
                  else
                    ++this->relationshipMatrix(indexJ, indexI);
                }
            }
        }
    }
}

Cluster::Cluster()
{
  // noop
}

Cluster::Cluster(std::string const &filepath)
{
  this->load(filepath);
}

Cluster::Cluster(std::vector<pcl::PointIndicesPtr> const &clusters, size_t pointIndexCount)
{
  this->clusters = clusters;
  this->pointIndexCount = pointIndexCount;
}

void Cluster::save(std::string filepath, std::string comment) const
{
  // binary mode to disable eol conversion (windows)
  std::ofstream file(filepath.c_str(), std::ifstream::binary);

  file << "version 1" << '\n';
  file << "pointIndexCount " << this->pointIndexCount << '\n';

  // add comments
  if (comment.size() > 0)
    {
      std::string escapedComments = boost::replace_all_copy(comment, "\n", "\n# ");
      file << "# " << escapedComments << '\n';
    }

  for (size_t i=0; i < this->clusters.size(); i++)
    {
      pcl::PointIndicesConstPtr const cluster = this->clusters[i];
      for (std::vector<int>::const_iterator indexIt = cluster->indices.begin(); indexIt != cluster->indices.end(); ++indexIt)
        {
          file << *indexIt;

          if (std::distance(indexIt, cluster->indices.end()) != 1)
            file << ",";
        }

      file << '\n';
    }
}

void Cluster::load(std::string filepath)
{
  this->clusters.resize(0);

  std::ifstream file(filepath.c_str());
  size_t version;
  std::string tmp;

  file >> tmp;

  if (tmp != "version")
    throw std::runtime_error("Illegal file-format! (expected \"version\")");

  file >> version;

  file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  if (version == 1)
    {
      file >> tmp;

      if (tmp != "pointIndexCount")
        throw std::runtime_error("Illegal file-format! (expected \"pointIndexCount\")");

      file >> this->pointIndexCount;

      file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

      while (!file.eof())
        {
          char nextChar = file.peek();

          if (nextChar == EOF)
            break;
          else if (nextChar == '#') // ignore comment-lines
            file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
          else
            {
              pcl::PointIndicesPtr pointIndicesPtr(new pcl::PointIndices());

              while (!file.eof())
                {
                  char nextChar = file.peek();

                  if (nextChar == EOF || nextChar == '\n')
                    {
                      file.ignore();
                      break;
                    }
                  else if (nextChar == '#') // ignore comment-lines
                    file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                  else if (nextChar == ',') // ignore comma
                    file.ignore();
                  else
                    {
                      int index;
                      file >> index;
                      pointIndicesPtr->indices.push_back(index);
                    }
                }

              this->clusters.push_back(pointIndicesPtr);
            }
        }
    }
  else
    throw std::runtime_error("Unsupported version!");
}

std::vector<pcl::PointIndicesPtr> const &Cluster::getClusters() const
{
  return this->clusters;
}

size_t Cluster::getPointIndexCount() const
{
  return this->pointIndexCount;
}

int Cluster::operator-(Cluster const &rhs) const
{
  if (this->getPointIndexCount() != rhs.getPointIndexCount())
    throw std::runtime_error("pointIndexCount has to be identical!");

  this->calculateRelationshipMatrixIfNecessary();
  rhs.calculateRelationshipMatrixIfNecessary();

  return (this->relationshipMatrix - rhs.relationshipMatrix).abs().sum();
}
