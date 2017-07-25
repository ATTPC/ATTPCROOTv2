#include "ATHierarchicalClusteringCluster.hh"

#include <fstream>
#include <limits>
#include <cctype>

#include <boost/algorithm/string/replace.hpp>

void ATHierarchicalClusteringCluster::CalculateRelationshipMatrixIfNecessary() const
{
    if (this->_relationshipMatrix.size() == 0)
    {
        this->_relationshipMatrix = Eigen::ArrayXXi(this->_pointIndexCount, this->_pointIndexCount);
        this->_relationshipMatrix.fill(0);

        for (std::vector<size_t> const &indices : this->GetClusters())
        {
            size_t indicesSize = indices.size();

            for (size_t i = 0; i < indicesSize; ++i)
            {
                int indexI = indices[i];

                ++this->_relationshipMatrix(indexI, indexI);

                for (size_t j = i + 1; j < indicesSize; ++j)
                {
                    int indexJ = indices[j];

                    if(indexI < indexJ)
                        ++this->_relationshipMatrix(indexI, indexJ);
                    else
                        ++this->_relationshipMatrix(indexJ, indexI);
                }
            }
        }
    }
}

ATHierarchicalClusteringCluster::ATHierarchicalClusteringCluster()
{
    // noop
}

ATHierarchicalClusteringCluster::ATHierarchicalClusteringCluster(std::string const &filepath)
{
    this->Load(filepath);
}

ATHierarchicalClusteringCluster::ATHierarchicalClusteringCluster(std::vector<std::vector<size_t>> const &clusters, size_t pointIndexCount)
{
    this->_clusters = clusters;
    this->_pointIndexCount = pointIndexCount;
}

void ATHierarchicalClusteringCluster::Save(std::string filepath, std::string comment) const
{
    // binary mode to disable eol conversion (windows)
    std::ofstream file(filepath, std::ifstream::binary);

    file << "version 1" << '\n';
    file << "pointIndexCount " << this->_pointIndexCount << '\n';

    // add comments
    if (comment.size() > 0)
    {
        std::string escapedComments = boost::replace_all_copy(comment, "\n", "\n# ");
        file << "# " << escapedComments << '\n';
    }

    for (std::vector<size_t> const &indices : this->GetClusters())
    {
        for (auto indexIt = indices.cbegin(); indexIt != indices.cend(); ++indexIt)
        {
            file << *indexIt;

            if (std::distance(indexIt, indices.cend()) != 1)
                file << ",";
        }

        file << '\n';
    }
}

void ATHierarchicalClusteringCluster::Load(std::string filepath)
{
    this->_clusters.resize(0);

    std::ifstream file(filepath);
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

        file >> this->_pointIndexCount;

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
                std::vector<size_t> pointIndices;

                while (!file.eof())
                {
                    char nextCharInner = file.peek();

                    if (nextCharInner == EOF || nextCharInner == '\n')
                    {
                        file.ignore();
                        break;
                    }
                    else if (nextCharInner == '#') // ignore comment-lines
                        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    else if (nextCharInner == ',') // ignore comma
                        file.ignore();
                    else
                    {
                        size_t index;
                        file >> index;
                        pointIndices.push_back(index);
                    }
                }

                this->_clusters.push_back(pointIndices);
            }
        }
    }
    else
        throw std::runtime_error("Unsupported version!");
}

std::vector<std::vector<size_t>> const &ATHierarchicalClusteringCluster::GetClusters() const
{
    return this->_clusters;
}

size_t ATHierarchicalClusteringCluster::GetPointIndexCount() const
{
    return this->_pointIndexCount;
}

int ATHierarchicalClusteringCluster::operator-(ATHierarchicalClusteringCluster const &rhs) const
{
    if (this->GetPointIndexCount() != rhs.GetPointIndexCount())
        throw std::runtime_error("PointIndexCount has to be identical!");

    this->CalculateRelationshipMatrixIfNecessary();
    rhs.CalculateRelationshipMatrixIfNecessary();

    return (this->_relationshipMatrix - rhs._relationshipMatrix).abs().sum();
}
