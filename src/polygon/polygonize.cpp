/*
 * Filename: d:\repo\linsar\src\polygon\polygonize.cpp
 * Path: d:\repo\linsar\src\polygon
 * Created Date: Friday, December 27th 2019, 2:57:46 pm
 * Author: Han Hu
 * 栅格转矢量
 * Copyright (c) 2019 Vrlab
 */
#include "polygonize.h"

namespace h2o {
#define GP_NODATA_MARKER 128

/************************************************************************/
/* ==================================================================== */
/*                               RPolygon                               */
/*                                                                      */
/*      This is a helper class to hold polygons while they are being    */
/*      formed in memory, and to provide services to coalesce a much    */
/*      of edge sections into complete rings.                           */
/* ==================================================================== */
/************************************************************************/

class RPolygon {
public:
    double dfPolyValue = 0.0;
    int nLastLineUpdated = -1;

    std::vector<std::vector<int>> aanXY;

    explicit RPolygon(double dfValue) : dfPolyValue(dfValue) {}

    void AddSegment(int x1, int y1, int x2, int y2);
    void Dump() const;
    void Coalesce();
    void Merge(int iBaseString, int iSrcString, int iDirection);
};

PolygonRings EmitPolygonToLayer(RPolygon *poRPoly) {
    /* -------------------------------------------------------------------- */
    /*      Turn bits of lines into coherent rings.                         */
    /* -------------------------------------------------------------------- */
    poRPoly->Coalesce();

    PolygonRings rings;

    for (size_t iString = 0; iString < poRPoly->aanXY.size(); iString++) {
        std::vector<int> &anString = poRPoly->aanXY[iString];
        PolygonRing ring(static_cast<int>(anString.size()) / 2);
        // We go last to first to ensure the linestring is allocated to
        // the proper size on the first try.
        for (int iVert = static_cast<int>(anString.size()) / 2 - 1; iVert >= 0; iVert--) {
            const int nPixelX = anString[iVert * 2];
            const int nPixelY = anString[iVert * 2 + 1];

            ring[iVert].x() = nPixelX;
            ring[iVert].y() = nPixelY;
        }
        rings.push_back(ring);
    }
    return rings;
}

/************************************************************************/
/*                                Dump()                                */
/************************************************************************/
void RPolygon::Dump() const {
    /*ok*/ printf("RPolygon: Value=%g, LastLineUpdated=%d\n", dfPolyValue, nLastLineUpdated);

    for (size_t iString = 0; iString < aanXY.size(); iString++) {
        const std::vector<int> &anString = aanXY[iString];

        /*ok*/ printf("  String %d:\n", static_cast<int>(iString));
        for (size_t iVert = 0; iVert < anString.size(); iVert += 2) {
            /*ok*/ printf("    (%d,%d)\n", anString[iVert], anString[iVert + 1]);
        }
    }
}

/************************************************************************/
/*                              Coalesce()                              */
/************************************************************************/

void RPolygon::Coalesce()

{
    /* -------------------------------------------------------------------- */
    /*      Iterate over loops starting from the first, trying to merge     */
    /*      other segments into them.                                       */
    /* -------------------------------------------------------------------- */
    for (size_t iBaseString = 0; iBaseString < aanXY.size(); iBaseString++) {
        std::vector<int> &anBase = aanXY[iBaseString];
        bool bMergeHappened = true;

        /* -------------------------------------------------------------------- */
        /*      Keep trying to merge the following strings into our target      */
        /*      "base" string till we have tried them all once without any      */
        /*      mergers.                                                        */
        /* -------------------------------------------------------------------- */
        while (bMergeHappened) {
            bMergeHappened = false;

            /* -------------------------------------------------------------------- */
            /*      Loop over the following strings, trying to find one we can      */
            /*      merge onto the end of our base string.                          */
            /* -------------------------------------------------------------------- */
            for (size_t iString = iBaseString + 1; iString < aanXY.size(); iString++) {
                std::vector<int> &anString = aanXY[iString];

                if (anBase[anBase.size() - 2] == anString[0] && anBase.back() == anString[1]) {
                    Merge(static_cast<int>(iBaseString), static_cast<int>(iString), 1);
                    bMergeHappened = true;
                } else if (anBase[anBase.size() - 2] == anString[anString.size() - 2] &&
                           anBase.back() == anString.back()) {
                    Merge(static_cast<int>(iBaseString), static_cast<int>(iString), -1);
                    bMergeHappened = true;
                }
            }
        }

        // At this point our loop *should* be closed!
        CHECK(anBase[0] == anBase[anBase.size() - 2] && anBase[1] == anBase.back());
    }
}

/************************************************************************/
/*                               Merge()                                */
/************************************************************************/
void RPolygon::Merge(int iBaseString, int iSrcString, int iDirection) {
    std::vector<int> &anBase = aanXY[iBaseString];
    std::vector<int> &anString = aanXY[iSrcString];
    int iStart = 1;
    int iEnd = -1;

    if (iDirection == 1) {
        iEnd = static_cast<int>(anString.size()) / 2;
    } else {
        iStart = static_cast<int>(anString.size()) / 2 - 2;
    }

    for (int i = iStart; i != iEnd; i += iDirection) {
        anBase.push_back(anString[i * 2 + 0]);
        anBase.push_back(anString[i * 2 + 1]);
    }

    if (iSrcString < static_cast<int>(aanXY.size()) - 1)
        aanXY[iSrcString] = aanXY[aanXY.size() - 1];

    const size_t nSize = aanXY.size();
    aanXY.resize(nSize - 1);
}

/************************************************************************/
/*                             AddSegment()                             */
/************************************************************************/

void RPolygon::AddSegment(int x1, int y1, int x2, int y2) {
    nLastLineUpdated = std::max(y1, y2);

    /* -------------------------------------------------------------------- */
    /*      Is there an existing string ending with this?                   */
    /* -------------------------------------------------------------------- */
    for (size_t iString = 0; iString < aanXY.size(); iString++) {
        std::vector<int> &anString = aanXY[iString];
        const size_t nSSize = anString.size();

        if (anString[nSSize - 2] == x1 && anString[nSSize - 1] == y1) {
            std::swap(x1, x2);
            std::swap(y1, y2);
        }

        if (anString[nSSize - 2] == x2 && anString[nSSize - 1] == y2) {
            // We are going to add a segment, but should we just extend
            // an existing segment already going in the right direction?

            const int nLastLen = std::max(std::abs(anString[nSSize - 4] - anString[nSSize - 2]),
                                          std::abs(anString[nSSize - 3] - anString[nSSize - 1]));

            if (nSSize >= 4 &&
                (anString[nSSize - 4] - anString[nSSize - 2] == (anString[nSSize - 2] - x1) * nLastLen) &&
                (anString[nSSize - 3] - anString[nSSize - 1] == (anString[nSSize - 1] - y1) * nLastLen)) {
                anString.pop_back();
                anString.pop_back();
            }

            anString.push_back(x1);
            anString.push_back(y1);
            return;
        }
    }

    /* -------------------------------------------------------------------- */
    /*      Create a new string.                                            */
    /* -------------------------------------------------------------------- */
    const size_t nSize = aanXY.size();
    aanXY.resize(nSize + 1);
    std::vector<int> &anString = aanXY[nSize];

    anString.push_back(x1);
    anString.push_back(y1);
    anString.push_back(x2);
    anString.push_back(y2);

    return;
}

/************************************************************************/
/* ==================================================================== */
/*     End of RPolygon                                                  */
/* ==================================================================== */
/************************************************************************/

/************************************************************************/
/*                              AddEdges()                              */
/*                                                                      */
/*      Examine one pixel and compare to its neighbour above            */
/*      (previous) and right.  If they are different polygon ids        */
/*      then add the pixel edge to this polygon and the one on the      */
/*      other side of the edge.                                         */
/************************************************************************/

static void AddEdges(int *panThisLineId, int *panLastLineId, int *panPolyIdMap, uint8_t *panPolyValue,
                     RPolygon **papoPoly, int iX, int iY)

{
    // TODO(schwehr): Simplify these three vars.
    int nThisId = panThisLineId[iX];
    if (nThisId != -1)
        nThisId = panPolyIdMap[nThisId];
    int nRightId = panThisLineId[iX + 1];
    if (nRightId != -1)
        nRightId = panPolyIdMap[nRightId];
    int nPreviousId = panLastLineId[iX];
    if (nPreviousId != -1)
        nPreviousId = panPolyIdMap[nPreviousId];

    const int iXReal = iX - 1;

    if (nThisId != nPreviousId) {
        if (nThisId != -1) {
            if (papoPoly[nThisId] == nullptr)
                papoPoly[nThisId] = new RPolygon(panPolyValue[nThisId]);

            papoPoly[nThisId]->AddSegment(iXReal, iY, iXReal + 1, iY);
        }
        if (nPreviousId != -1) {
            if (papoPoly[nPreviousId] == nullptr)
                papoPoly[nPreviousId] = new RPolygon(panPolyValue[nPreviousId]);

            papoPoly[nPreviousId]->AddSegment(iXReal, iY, iXReal + 1, iY);
        }
    }

    if (nThisId != nRightId) {
        if (nThisId != -1) {
            if (papoPoly[nThisId] == nullptr)
                papoPoly[nThisId] = new RPolygon(panPolyValue[nThisId]);

            papoPoly[nThisId]->AddSegment(iXReal + 1, iY, iXReal + 1, iY + 1);
        }

        if (nRightId != -1) {
            if (papoPoly[nRightId] == nullptr)
                papoPoly[nRightId] = new RPolygon(panPolyValue[nRightId]);

            papoPoly[nRightId]->AddSegment(iXReal + 1, iY, iXReal + 1, iY + 1);
        }
    }
}

/************************************************************************/
/*                          Polygon Enumerator                          */
/************************************************************************/

class GDALRasterPolygonEnumeratorT {
private:
    void MergePolygon(int nSrcId, int nDstId);
    int NewPolygon(uint8_t nValue);

public: // these are intended to be readonly.
    std::vector<int> panPolyIdMap;
    std::vector<uint8_t> panPolyValue;

    int nNextPolygonId = 0;
    int nPolyAlloc = 0;

    int nConnectedness = 0;

public:
    explicit GDALRasterPolygonEnumeratorT(int nConnectedness = 4);
    ~GDALRasterPolygonEnumeratorT();

    void ProcessLine(uint8_t *panLastLineVal, uint8_t *panThisLineVal, int *panLastLineId, int *panThisLineId,
                     int nXSize);
    void CompleteMerges();

    void Clear();
};

GDALRasterPolygonEnumeratorT::GDALRasterPolygonEnumeratorT(int nConnectednessIn) : nConnectedness(nConnectednessIn) {
    CHECK(nConnectedness == 4 || nConnectedness == 8);
}

/************************************************************************/
/*                    ~GDALRasterPolygonEnumeratorT()                    */
/************************************************************************/

GDALRasterPolygonEnumeratorT::~GDALRasterPolygonEnumeratorT() { Clear(); }

/************************************************************************/
/*                               Clear()                                */
/************************************************************************/

void GDALRasterPolygonEnumeratorT::Clear() {
    nNextPolygonId = 0;
    nPolyAlloc = 0;
}

/************************************************************************/
/*                            MergePolygon()                            */
/*                                                                      */
/*      Update the polygon map to indicate the merger of two polygons.  */
/************************************************************************/

void GDALRasterPolygonEnumeratorT::MergePolygon(int nSrcId, int nDstIdInit) {
    // Figure out the final dest id.
    int nDstIdFinal = nDstIdInit;
    while (panPolyIdMap[nDstIdFinal] != nDstIdFinal)
        nDstIdFinal = panPolyIdMap[nDstIdFinal];

    // Map the whole intermediate chain to it.
    int nDstIdCur = nDstIdInit;
    while (panPolyIdMap[nDstIdCur] != nDstIdCur) {
        int nNextDstId = panPolyIdMap[nDstIdCur];
        panPolyIdMap[nDstIdCur] = nDstIdFinal;
        nDstIdCur = nNextDstId;
    }

    // And map the whole source chain to it too (can be done in one pass).
    while (panPolyIdMap[nSrcId] != nSrcId) {
        int nNextSrcId = panPolyIdMap[nSrcId];
        panPolyIdMap[nSrcId] = nDstIdFinal;
        nSrcId = nNextSrcId;
    }
    panPolyIdMap[nSrcId] = nDstIdFinal;
}

/************************************************************************/
/*                             NewPolygon()                             */
/*                                                                      */
/*      Allocate a new polygon id, and reallocate the polygon maps      */
/*      if needed.                                                      */
/************************************************************************/

int GDALRasterPolygonEnumeratorT::NewPolygon(uint8_t nValue)

{
    const int nPolyId = nNextPolygonId;

    if (nNextPolygonId >= nPolyAlloc) {
        nPolyAlloc = nPolyAlloc * 2 + 20;
        panPolyIdMap.resize(nPolyAlloc, 0);
        panPolyValue.resize(nPolyAlloc, 0);
    }

    nNextPolygonId++;

    panPolyIdMap[nPolyId] = nPolyId;
    panPolyValue[nPolyId] = nValue;

    return nPolyId;
}

/************************************************************************/
/*                           CompleteMerges()                           */
/*                                                                      */
/*      Make a pass through the maps, ensuring every polygon id         */
/*      points to the final id it should use, not an intermediate       */
/*      value.                                                          */
/************************************************************************/

void GDALRasterPolygonEnumeratorT::CompleteMerges() {
    int nFinalPolyCount = 0;

    for (int iPoly = 0; iPoly < nNextPolygonId; iPoly++) {
        // Figure out the final id.
        int nId = panPolyIdMap[iPoly];
        while (nId != panPolyIdMap[nId]) {
            nId = panPolyIdMap[nId];
        }

        // Then map the whole intermediate chain to it.
        int nIdCur = panPolyIdMap[iPoly];
        panPolyIdMap[iPoly] = nId;
        while (nIdCur != panPolyIdMap[nIdCur]) {
            int nNextId = panPolyIdMap[nIdCur];
            panPolyIdMap[nIdCur] = nId;
            nIdCur = nNextId;
        }

        if (panPolyIdMap[iPoly] == iPoly)
            nFinalPolyCount++;
    }
}

/************************************************************************/
/*                            ProcessLine()                             */
/*                                                                      */
/*      Assign ids to polygons, one line at a time.                     */
/************************************************************************/

void GDALRasterPolygonEnumeratorT::ProcessLine(uint8_t *panLastLineVal, uint8_t *panThisLineVal, int *panLastLineId,
                                               int *panThisLineId, int nXSize)

{
    /* -------------------------------------------------------------------- */
    /*      Special case for the first line.                                */
    /* -------------------------------------------------------------------- */
    if (panLastLineVal == nullptr) {
        for (int i = 0; i < nXSize; i++) {
            if (panThisLineVal[i] == GP_NODATA_MARKER) {
                panThisLineId[i] = -1;
            } else if (i == 0 || !(panThisLineVal[i] == panThisLineVal[i - 1])) {
                panThisLineId[i] = NewPolygon(panThisLineVal[i]);
            } else {
                panThisLineId[i] = panThisLineId[i - 1];
            }
        }

        return;
    }

    /* -------------------------------------------------------------------- */
    /*      Process each pixel comparing to the previous pixel, and to      */
    /*      the last line.                                                  */
    /* -------------------------------------------------------------------- */
    for (int i = 0; i < nXSize; i++) {
        if (panThisLineVal[i] == GP_NODATA_MARKER) {
            panThisLineId[i] = -1;
        } else if (i > 0 && (panThisLineVal[i] == panThisLineVal[i - 1])) {
            panThisLineId[i] = panThisLineId[i - 1];

            if ((panLastLineVal[i] == panThisLineVal[i]) &&
                (panPolyIdMap[panLastLineId[i]] != panPolyIdMap[panThisLineId[i]])) {
                MergePolygon(panLastLineId[i], panThisLineId[i]);
            }

            if (nConnectedness == 8 && (panLastLineVal[i - 1] == panThisLineVal[i]) &&
                (panPolyIdMap[panLastLineId[i - 1]] != panPolyIdMap[panThisLineId[i]])) {
                MergePolygon(panLastLineId[i - 1], panThisLineId[i]);
            }

            if (nConnectedness == 8 && i < nXSize - 1 && (panLastLineVal[i + 1] == panThisLineVal[i]) &&
                (panPolyIdMap[panLastLineId[i + 1]] != panPolyIdMap[panThisLineId[i]])) {
                MergePolygon(panLastLineId[i + 1], panThisLineId[i]);
            }
        } else if (panLastLineVal[i] == panThisLineVal[i]) {
            panThisLineId[i] = panLastLineId[i];
        } else if (i > 0 && nConnectedness == 8 && (panLastLineVal[i - 1] == panThisLineVal[i])) {
            panThisLineId[i] = panLastLineId[i - 1];

            if (i < nXSize - 1 && (panLastLineVal[i + 1] == panThisLineVal[i]) &&
                (panPolyIdMap[panLastLineId[i + 1]] != panPolyIdMap[panThisLineId[i]])) {
                MergePolygon(panLastLineId[i + 1], panThisLineId[i]);
            }
        } else if (i < nXSize - 1 && nConnectedness == 8 && (panLastLineVal[i + 1] == panThisLineVal[i])) {
            panThisLineId[i] = panLastLineId[i + 1];
        } else {
            panThisLineId[i] = NewPolygon(panThisLineVal[i]);
        }
    }
}

/************************************************************************/
/*                           GDALPolygonizeT()                          */
/************************************************************************/

PolygonRings GDALPolygonize(const cv::Mat &hSrcBand, const cv::Mat &hMaskBand) {
    PolygonRings rings;

    const int nConnectedness = 8;

    /* -------------------------------------------------------------------- */
    /*      Allocate working buffers.                                       */
    /* -------------------------------------------------------------------- */
    const int nXSize = hSrcBand.cols;
    const int nYSize = hSrcBand.rows;

    std::vector<int> panLastLineId(nXSize + 2);
    std::vector<int> panThisLineId(nXSize + 2);
    std::vector<uint8_t> panThisLineVal(nXSize + 2);
    std::vector<uint8_t> panLastLineVal(nXSize + 2);
    std::vector<uint8_t> pabyMaskLine(nXSize);

    /* -------------------------------------------------------------------- */
    /*      The first pass over the raster is only used to build up the     */
    /*      polygon id map so we will know in advance what polygons are     */
    /*      what on the second pass.                                        */
    /* -------------------------------------------------------------------- */
    GDALRasterPolygonEnumeratorT oFirstEnum(nConnectedness);

    for (int iY = 0; iY < nYSize; iY++) {
        for (int x = 0; x < nXSize; ++x) {
            panThisLineVal[x] = hSrcBand.ptr<uint8_t>(iY)[x];
        }
        for (int x = 0; x < nXSize; ++x) {
            pabyMaskLine[x] = hMaskBand.ptr<uint8_t>(iY)[x];
            if (pabyMaskLine[x] == 0) {
                panThisLineVal[x] = GP_NODATA_MARKER;
            }
        }

        if (iY == 0)
            oFirstEnum.ProcessLine(nullptr, panThisLineVal.data(), nullptr, panThisLineId.data(), nXSize);
        else
            oFirstEnum.ProcessLine(panLastLineVal.data(), panThisLineVal.data(), panLastLineId.data(),
                                   panThisLineId.data(), nXSize);

        // Swap lines.
        std::swap(panLastLineVal, panThisLineVal);
        std::swap(panLastLineId, panThisLineId);
    }

    /* -------------------------------------------------------------------- */
    /*      Make a pass through the maps, ensuring every polygon id         */
    /*      points to the final id it should use, not an intermediate       */
    /*      value.                                                          */
    /* -------------------------------------------------------------------- */
    oFirstEnum.CompleteMerges();

    /* -------------------------------------------------------------------- */
    /*      Initialize ids to -1 to serve as a nodata value for the         */
    /*      previous line, and past the beginning and end of the            */
    /*      scanlines.                                                      */
    /* -------------------------------------------------------------------- */
    panThisLineId[0] = -1;
    panThisLineId[nXSize + 1] = -1;

    for (int iX = 0; iX < nXSize + 2; iX++)
        panLastLineId[iX] = -1;

    /* -------------------------------------------------------------------- */
    /*      We will use a new enumerator for the second pass primarily      */
    /*      so we can preserve the first pass map.                          */
    /* -------------------------------------------------------------------- */
    GDALRasterPolygonEnumeratorT oSecondEnum(nConnectedness);
    std::vector<RPolygon *> papoPoly(oFirstEnum.nNextPolygonId);

    /* ==================================================================== */
    /*      Second pass during which we will actually collect polygon       */
    /*      edges as geometries.                                            */
    /* ==================================================================== */
    for (int iY = 0; iY < nYSize + 1; iY++) {
        /* -------------------------------------------------------------------- */
        /*      Read the image data.                                            */
        /* -------------------------------------------------------------------- */
        if (iY < nYSize) {
            for (int x = 0; x < nXSize; ++x) {
                panThisLineVal[x] = hSrcBand.ptr<uint8_t>(iY)[x];
            }
            for (int x = 0; x < nXSize; ++x) {
                pabyMaskLine[x] = hMaskBand.ptr<uint8_t>(iY)[x];
                if (pabyMaskLine[x] == 0) {
                    panThisLineVal[x] = GP_NODATA_MARKER;
                }
            }
        }

        /* -------------------------------------------------------------------- */
        /*      Determine what polygon the various pixels belong to (redoing    */
        /*      the same thing done in the first pass above).                   */
        /* -------------------------------------------------------------------- */
        if (iY == nYSize) {
            for (int iX = 0; iX < nXSize + 2; iX++)
                panThisLineId[iX] = -1;
        } else if (iY == 0) {
            oSecondEnum.ProcessLine(nullptr, panThisLineVal.data(), nullptr, panThisLineId.data() + 1, nXSize);
        } else {
            oSecondEnum.ProcessLine(panLastLineVal.data(), panThisLineVal.data(), panLastLineId.data() + 1,
                                    panThisLineId.data() + 1, nXSize);
        }

        /* -------------------------------------------------------------------- */
        /*      Add polygon edges to our polygon list for the pixel             */
        /*      boundaries within and above this line.                          */
        /* -------------------------------------------------------------------- */
        for (int iX = 0; iX < nXSize + 1; iX++) {
            AddEdges(panThisLineId.data(), panLastLineId.data(), oFirstEnum.panPolyIdMap.data(),
                     oFirstEnum.panPolyValue.data(), papoPoly.data(), iX, iY);
        }

        /* -------------------------------------------------------------------- */
        /*      Periodically we scan out polygons and write out those that      */
        /*      haven't been added to on the last line as we can be sure        */
        /*      they are complete.                                              */
        /* -------------------------------------------------------------------- */
        if (iY % 8 == 7) {
            for (int iX = 0; iX < oSecondEnum.nNextPolygonId; iX++) {
                if (papoPoly[iX] && papoPoly[iX]->nLastLineUpdated < iY - 1) {
                    PolygonRings rings2 = EmitPolygonToLayer(papoPoly[iX]);
                    rings.insert(rings.end(), begin(rings2), end(rings2));
                    delete papoPoly[iX];
                    papoPoly[iX] = nullptr;
                }
            }
        }

        /* -------------------------------------------------------------------- */
        /*      Swap pixel value, and polygon id lines to be ready for the      */
        /*      next line.                                                      */
        /* -------------------------------------------------------------------- */
        std::swap(panLastLineVal, panThisLineVal);
        std::swap(panLastLineId, panThisLineId);
    }

    /* -------------------------------------------------------------------- */
    /*      Make a cleanup pass for all unflushed polygons.                 */
    /* -------------------------------------------------------------------- */
    for (int iX = 0; iX < oSecondEnum.nNextPolygonId; iX++) {
        if (papoPoly[iX]) {
            PolygonRings rings2 = EmitPolygonToLayer(papoPoly[iX]);
            rings.insert(rings.end(), begin(rings2), end(rings2));
            delete papoPoly[iX];
            papoPoly[iX] = nullptr;
        }
    }
    return rings;
}

PolygonRings polygonize(const cv::Mat &mask) { return GDALPolygonize(mask, mask); }
} // namespace h2o
