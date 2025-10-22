#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <cmath>
#include <algorithm> // std::sort için eklendi
#include <cstdio>    // snprintf için
#include <SFML/Graphics.hpp>
#include <curl/curl.h>

// DÜZELTME: TTF font eklentisine artık ihtiyaç olmadığı için kaldırılabilir veya kalabilir.
// Temizlik açısından kaldırıyorum.
// #include <allegro5/allegro_ttf.h>

// M_PI sabitini <cmath> içinde kullanılabilir hale getirmek için
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// === STRUCT'LAR (DEĞİŞİKLİK YOK) ===
struct point {
    double x = {0.0};
    double y = {0.0};
};

struct toml {
    double angle_min{0.0};
    double angle_max{0.0};
    double angle_increment{0.0};
    double range_min{0.00};
    double range_max{0.00};
    std::vector<double> ranges;
};

struct parameters {
    double segmentationDistanceThreshHold = 0.1;
    double lineFitThreshHold = 0.05;
    int minPointsForline = 8;
};

struct lineSegment {
    int i0;
    int i1;
};

struct Line {
    point start;
    point end;
};

// === ALGORİTMA FONKSİYONLARI (DEĞİŞİKLİK YOK) ===
// (Bu fonksiyonların tamamı sizin yazdığınız gibi korunmuştur)

size_t WriteToFile(void* contents, size_t size, size_t nmemb, void* userp) {
    std::ofstream* outFile = static_cast<std::ofstream*>(userp);
    size_t totalSize = size * nmemb;
    outFile->write(static_cast<char*>(contents), totalSize);
    return totalSize;
}

bool downloadTOML(const std::string& url, const std::string& outputPath) {
    CURL* curl;
    CURLcode res;
    curl = curl_easy_init();
    if (!curl) {
        std::cerr << "curl başlatılamadı!\n";
        return false;
    }

    std::ofstream outFile(outputPath, std::ios::binary);
    if (!outFile.is_open()) {
        std::cerr << "Dosya oluşturulamadı: " << outputPath << "\n";
        curl_easy_cleanup(curl);
        return false;
    }

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteToFile);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &outFile);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L); // yönlendirmeleri takip et
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "libcurl-agent/1.0");

    res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        std::cerr << "İndirme hatası: " << curl_easy_strerror(res) << "\n";
        curl_easy_cleanup(curl);
        return false;
    }

    curl_easy_cleanup(curl);
    outFile.close();
    return true;
}

std::vector<point> filtrele(const toml* t) {
    std::vector<point> result;
    result.reserve(t->ranges.size());
    double angle = t->angle_min;
    for (size_t i = 0; i < t->ranges.size(); i++) {
        double range = t->ranges[i];
        if (range >= t->range_min && range <= t->range_max) {
            double x = range * cos(angle);
            double y = range * sin(angle);
            result.push_back(point{x, y});
        }
        angle += t->angle_increment;
    }
    std::cout << "Filtrelenmis nokta sayisi: " << result.size() << std::endl;
    return result;
}

double mesafeFarkiHesapla(const point& p0, const point& p1) {
    return std::hypot(p1.y - p0.y, p1.x - p0.x);
}

std::vector<std::vector<point>> segmentlereAyir(const std::vector<point>& p, const double distance) {
    std::vector<std::vector<point>> result;
    if (p.empty()) return result;
    result.push_back({p[0]});
    for (size_t i = 1; i < p.size(); i++) {
        const point& mevcutNokta = p[i];
        const point& OncekiNokta = p[i - 1];
        if (mesafeFarkiHesapla(OncekiNokta, mevcutNokta) < distance) {
            result.back().push_back(mevcutNokta);
        } else {
            result.push_back({mevcutNokta});
        }
    }
    return result;
}

double noktaninDoğruyaUzakligi(const point& p, const point& line1, const point& line2) {
    double dx = line2.x - line1.x;
    double dy = line2.y - line1.y;
    if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) {
        return mesafeFarkiHesapla(p, line1);
    }
    double pay = std::abs(dy * p.x - dx * p.y + line2.x * line1.y - line2.y * line1.x);
    double payda = std::sqrt(dx * dx + dy * dy);
    return (pay / payda);
}

void recursiveSplit(const std::vector<point>& segment, int i0, int i1, const parameters& p, std::vector<lineSegment>& result_segments) {
    if (i1 <= i0 + 1) {
        if (i1 > i0) result_segments.push_back({i0, i1});
        return;
    }
    const point& line1 = segment[i0];
    const point& line2 = segment[i1];
    double maxDistance = 0.0;
    int splitIndex = -1;
    for (int i = i0 + 1; i < i1; i++) {
        double distance = noktaninDoğruyaUzakligi(segment[i], line1, line2);
        if (distance > maxDistance) {
            maxDistance = distance;
            splitIndex = i;
        }
    }
    if (maxDistance > p.lineFitThreshHold) {
        recursiveSplit(segment, i0, splitIndex, p, result_segments);
        recursiveSplit(segment, splitIndex, i1, p, result_segments);
    } else {
        result_segments.push_back({i0, i1});
    }
}

std::vector<Line> mergeSegments(const std::vector<point>& segment, const std::vector<lineSegment>& splitSegments, const parameters& p) {
    std::vector<Line> result;
    if (splitSegments.empty()) return result;
    int startIndex = splitSegments[0].i0;
    int endIndex = splitSegments[0].i1;
    for (size_t i = 0; i < splitSegments.size() - 1; i++) {
        endIndex = splitSegments[i].i1;
        int nextEndIndex = splitSegments[i + 1].i1;
        const point& line1 = segment[startIndex];
        const point& line2 = segment[nextEndIndex];
        bool can_merge = true;
        for (int j = startIndex; j <= nextEndIndex; j++) {
            if (noktaninDoğruyaUzakligi(segment[j], line1, line2) > p.lineFitThreshHold) {
                can_merge = false;
                break;
            }
        }
        if (!can_merge) {
            int pointCount = endIndex - startIndex + 1;
            if (pointCount >= p.minPointsForline) {
                result.push_back({segment[startIndex], segment[endIndex]});
            }
            startIndex = splitSegments[i + 1].i0;
        }
    }
    endIndex = splitSegments.back().i1;
    int pointCount = endIndex - startIndex + 1;
    if (pointCount >= p.minPointsForline) {
        result.push_back({segment[startIndex], segment[endIndex]});
    }
    return result;
}

std::vector<Line> findLinesWithSplitAndMerge(const std::vector<point>& segment, const parameters& p) {
    if (segment.size() < p.minPointsForline) {
        return {};
    }
    std::vector<lineSegment> split_segments;
    recursiveSplit(segment, 0, segment.size() - 1, p, split_segments);
    std::sort(split_segments.begin(), split_segments.end(), [](const lineSegment& a, const lineSegment& b) {
        return a.i0 < b.i0;
    });
    return mergeSegments(segment, split_segments, p);
}

// === YARDIMCI FONKSİYONLAR (DEĞİŞİKLİK YOK) ===

bool arePointsEqual(const point& p1, const point& p2, double epsilon = 1e-6) {
    return (std::abs(p1.x - p2.x) < epsilon) && (std::abs(p1.y - p2.y) < epsilon);
}

void draw_dashed_line(sf::RenderWindow& window, sf::Vector2f p1, sf::Vector2f p2, sf::Color color, float dash_len, float gap_len) {
    sf::Vector2f direction = p2 - p1;
    float total_length = std::hypot(direction.x, direction.y);
    if (total_length < 0.001f) return;

    sf::Vector2f unit_direction = direction / total_length;
    float current_pos = 0.0f;

    while (current_pos < total_length) {
        sf::Vector2f dash_start = p1 + unit_direction * current_pos;
        float end_pos = current_pos + dash_len;
        if (end_pos > total_length) end_pos = total_length;
        sf::Vector2f dash_end = p1 + unit_direction * end_pos;

        sf::Vertex line[] = { sf::Vertex(dash_start, color), sf::Vertex(dash_end, color) };
        window.draw(line, 2, sf::Lines);

        current_pos += dash_len + gap_len;
    }
}

double calculateLineAngleDeg(const Line& line) {
    double midX = (line.start.x + line.end.x) / 2.0;
    double midY = (line.start.y + line.end.y) / 2.0;
    double angle_rad = atan2(midY, midX);
    return angle_rad * 180.0 / M_PI;
}


int main() {
    toml t;
    parameters p;
   t.angle_min = 0.0;
    t.angle_max = 4.7;
    t.angle_increment = 0.0174533;
    t.range_min = 0.20;
    t.range_max = 3.00;

    t.ranges = {
         2.51, 2.50, 2.52, 2.51, 2.49, 2.53, 2.50, 2.51, 2.54, 2.48, 2.52, 2.50, 2.53, 2.49, 2.51, 2.55, 2.47, 2.52, 2.50, 2.54,
    2.48, 2.53, 2.49, 2.51, 2.56, 2.46, 2.52, 2.50, 2.55, 2.47, 2.53, 2.49, 2.51, 2.57, 2.45, 2.52, 2.50, 2.56, 2.46, 2.54,
    2.48, 2.53, 2.50, 2.57, 2.45, 2.52, 2.49, 2.56, 2.46, 2.54, 2.48, 2.53, 2.50, 2.58, 2.44, 2.52, 2.49, 2.57, 2.45, 2.55,
    2.47, 2.53, 2.50, 2.58, 2.44, 2.52, 2.49, 2.57, 2.45, 2.55, 2.47, 2.54, 2.50, 2.59, 2.43, 2.52, 2.49, 2.58, 2.44, 2.56,
    2.46, 2.54, 2.50, 2.59, 2.43, 2.53, 2.48, 2.58, 2.44, 2.56, 2.46, 2.54, 2.50, 2.60, 2.42, 2.53, 2.48, 2.59, 2.43, 2.57,
    2.45, 2.55, 2.50, 2.60, 2.42, 2.53, 2.48, 2.59, 2.43, 2.57, 2.45, 2.55, 2.51, 2.61, 2.41, 2.54, 2.47, 2.60, 2.42, 2.58,
    2.44, 2.56, 2.51, 2.61, 2.41, 2.54, 2.47, 2.60, 2.42, 2.58, 2.44, 2.56, 2.51, 2.62, 2.40, 2.55, 2.46, 2.61, 2.41, 2.59,
    2.43, 2.57, 2.51, 2.62, 2.40, 2.55, 2.46, 2.61, 2.41, 2.59, 2.43, 2.57, 2.52, 2.63, 2.39, 2.56, 2.45, 2.62, 2.40, 2.60,
    2.42, 2.58, 2.52, 2.63, 2.39, 2.56, 2.45, 2.62, 2.40, 2.60, 2.42, 2.58, 2.52, 2.64, 2.38, 2.57, 2.44, 2.63, 2.39, 2.61,
    2.41, 2.59, 2.53, 2.64, 2.38, 2.57, 2.44, 2.63, 2.39, 2.61, 2.41, 2.59, 2.53, 2.65, 2.37, 2.58, 2.43, 2.64, 2.38, 2.62,
    2.40, 2.60, 2.53, 2.65, 2.37, 2.58, 2.43, 2.64, 2.38, 2.62, 2.40, 2.60, 2.54, 2.66, 2.36, 2.59, 2.42, 2.65, 2.37, 2.63,
    2.39, 2.61, 2.54, 2.66, 2.36, 2.59, 2.42, 2.65, 2.37, 2.63, 2.39, 2.61, 2.54, 2.67, 2.35, 2.60, 2.41, 2.66, 2.36, 2.64,
    2.38, 2.62, 2.55, 2.67, 2.35,

    // Geçiş Bölgesi: Birkaç geçersiz ölçüm
    -1.0, 999.0, -1.0,

    // 2. Yüzey: Yaklaşık 1.8 metre uzaklıkta, daha pürüzsüz bir duvar (yaklaşık 300 nokta)
    1.80, 1.81, 1.79, 1.80, 1.82, 1.78, 1.80, 1.81, 1.79, 1.80, 1.82, 1.78, 1.81, 1.80, 1.79, 1.82, 1.78, 1.80, 1.81, 1.79,
    1.80, 1.82, 1.78, 1.81, 1.79, 1.80, 1.83, 1.77, 1.81, 1.80, 1.79, 1.82, 1.78, 1.81, 1.80, 1.79, 1.83, 1.77, 1.81, 1.80,
    1.79, 1.82, 1.78, 1.81, 1.80, 1.79, 1.83, 1.77, 1.81, 1.80, 1.79, 1.82, 1.78, 1.81, 1.80, 1.79, 1.84, 1.76, 1.82, 1.80,
    1.78, 1.83, 1.77, 1.81, 1.80, 1.79, 1.82, 1.78, 1.81, 1.80, 1.79, 1.84, 1.76, 1.82, 1.80, 1.78, 1.83, 1.77, 1.81, 1.80,
    1.79, 1.82, 1.78, 1.81, 1.80, 1.79, 1.84, 1.76, 1.82, 1.80, 1.78, 1.83, 1.77, 1.81, 1.80, 1.79, 1.85, 1.75, 1.83, 1.79,
    1.81, 1.80, 1.78, 1.84, 1.76, 1.82, 1.80, 1.78, 1.83, 1.77, 1.81, 1.80, 1.79, 1.85, 1.75, 1.83, 1.79, 1.81, 1.80, 1.78,
    1.84, 1.76, 1.82, 1.80, 1.78, 1.83, 1.77, 1.81, 1.80, 1.79, 1.85, 1.75, 1.83, 1.79, 1.81, 1.80, 1.78, 1.84, 1.76, 1.82,
    1.80, 1.78, 1.83, 1.77, 1.81, 1.80, 1.79, 1.86, 1.74, 1.84, 1.78, 1.82, 1.80, 1.78, 1.85, 1.75, 1.83, 1.79, 1.81, 1.80,
    1.78, 1.84, 1.76, 1.82, 1.80, 1.78, 1.83, 1.77, 1.81, 1.80, 1.79, 1.86, 1.74, 1.84, 1.78, 1.82, 1.80, 1.78, 1.85, 1.75,
    1.83, 1.79, 1.81, 1.80, 1.78, 1.84, 1.76, 1.82, 1.80, 1.78, 1.83, 1.77, 1.81, 1.80, 1.79, 1.86, 1.74, 1.84, 1.78, 1.82,
    1.80, 1.78, 1.85, 1.75, 1.83, 1.79, 1.81, 1.80, 1.78, 1.84, 1.76, 1.82, 1.80, 1.78, 1.83, 1.77, 1.81, 1.80, 1.79, 1.87,
    1.73, 1.85, 1.77, 1.83, 1.79, 1.81, 1.80, 1.78, 1.86, 1.74, 1.84, 1.78, 1.82, 1.80, 1.78, 1.85, 1.75, 1.83, 1.79, 1.81,
    1.80, 1.78, 1.84, 1.76, 1.82, 1.80, 1.78, 1.83, 1.77, 1.81, 1.80, 1.79, 1.87, 1.73, 1.85, 1.77, 1.83, 1.79, 1.81, 1.80,
    1.78, 1.86, 1.74, 1.84, 1.78, 1.82, 1.80, 1.78, 1.85, 1.75, 1.83, 1.79, 1.81, 1.80, 1.78, 1.84, 1.76, 1.82, 1.80, 1.78,
    1.83, 1.77, 1.81, 1.80, 1.79, 1.88, 1.72, 1.86, 1.76, 1.84, 1.78, 1.82, 1.80, 1.78, 1.87, 1.73, 1.85, 1.77, 1.83, 1.79,

    // Geçiş Bölgesi: Boşluk
    -1.0, -1.0, -1.0, -1.0,

    // 3. Yüzey: Yaklaşık 3.0 metre uzaklıkta, biraz daha gürültülü bir duvar (yaklaşık 200 nokta)
    3.02, 2.98, 3.03, 2.97, 3.04, 2.96, 3.05, 2.95, 3.06, 2.94, 3.07, 2.93, 3.08, 2.92, 3.09, 2.91, 3.10, 2.90, 3.11, 2.89,
    3.02, 2.98, 3.03, 2.97, 3.04, 2.96, 3.05, 2.95, 3.06, 2.94, 3.07, 2.93, 3.08, 2.92, 3.09, 2.91, 3.10, 2.90, 3.11, 2.89,
    3.03, 2.97, 3.04, 2.96, 3.05, 2.95, 3.06, 2.94, 3.07, 2.93, 3.08, 2.92, 3.09, 2.91, 3.10, 2.90, 3.12, 2.88, 3.04, 2.96,
    3.05, 2.95, 3.06, 2.94, 3.07, 2.93, 3.08, 2.92, 3.09, 2.91, 3.11, 2.89, 3.12, 2.88, 3.05, 2.95, 3.06, 2.94, 3.07, 2.93,
    3.08, 2.92, 3.10, 2.90, 3.11, 2.89, 3.12, 2.88, 3.13, 2.87, 3.06, 2.94, 3.07, 2.93, 3.08, 2.92, 3.10, 2.90, 3.11, 2.89,
    3.12, 2.88, 3.13, 2.87, 3.07, 2.93, 3.08, 2.92, 3.10, 2.90, 3.11, 2.89, 3.13, 2.87, 3.14, 2.86, 3.08, 2.92, 3.10, 2.90,
    3.11, 2.89, 3.13, 2.87, 3.14, 2.86, 3.09, 2.91, 3.11, 2.89, 3.13, 2.87, 3.14, 2.86, 3.15, 2.85, 3.10, 2.90, 3.12, 2.88,
    3.14, 2.86, 3.15, 2.85, 3.11, 2.89, 3.13, 2.87, 3.15, 2.85, 3.16, 2.84, 3.12, 2.88, 3.14, 2.86, 3.16, 2.84, 3.13, 2.87,
    3.15, 2.85, 3.17, 2.83, 3.14, 2.86, 3.16, 2.84, 3.18, 2.82, 3.15, 2.85, 3.17, 2.83, 3.19, 2.81, 3.16, 2.84, 3.18, 2.82,

    // Geçiş Bölgesi
    -999.0,

    // 4. Yüzey: Köşe veya uzaklaşan bir nesne (yaklaşık 250 nokta)
    2.00, 2.02, 2.04, 2.06, 2.08, 2.10, 2.12, 2.14, 2.16, 2.18, 2.20, 2.22, 2.24, 2.26, 2.28, 2.30, 2.32, 2.34, 2.36, 2.38,
    2.40, 2.42, 2.44, 2.46, 2.48, 2.50, 2.52, 2.54, 2.56, 2.58, 2.60, 2.62, 2.64, 2.66, 2.68, 2.70, 2.72, 2.74, 2.76, 2.78,
    2.80, 2.82, 2.84, 2.86, 2.88, 2.90, 2.92, 2.94, 2.96, 2.98, 3.00, 2.99, 2.97, 2.95, 2.93, 2.91, 2.89, 2.87, 2.85, 2.83,
    2.81, 2.79, 2.77, 2.75, 2.73, 2.71, 2.69, 2.67, 2.65, 2.63, 2.61, 2.59, 2.57, 2.55, 2.53, 2.51, 2.49, 2.47, 2.45, 2.43,
    2.41, 2.39, 2.37, 2.35, 2.33, 2.31, 2.29, 2.27, 2.25, 2.23, 2.21, 2.19, 2.17, 2.15, 2.13, 2.11, 2.09, 2.07, 2.05, 2.03,
    2.01, 1.99, 1.97, 1.95, 1.93, 1.91, 1.89, 1.87, 1.85, 1.83, 1.81, 1.79, 1.77, 1.75, 1.73, 1.71, 1.69, 1.67, 1.65, 1.63,
    1.61, 1.59, 1.57, 1.55, 1.53, 1.51, 1.49, 1.47, 1.45, 1.43, 1.41, 1.39, 1.37, 1.35, 1.33, 1.31, 1.29, 1.27, 1.25, 1.23,
    1.21, 1.19, 1.17, 1.15, 1.13, 1.11, 1.09, 1.07, 1.05, 1.03, 1.01, 0.99, 0.97, 0.95, 0.93, 0.91, 0.89, 0.87, 0.85, 0.83,
    0.81, 0.79, 0.77, 0.75, 0.73, 0.71, 0.69, 0.67, 0.65, 0.63, 0.61, 0.59, 0.57, 0.55, 0.53, 0.51, 0.49, 0.47, 0.45, 0.43,
    0.41, 0.39, 0.37, 0.35, 0.33, 0.31, 0.29, 0.27, 0.25, 0.23, 0.21, 0.20, 0.22, 0.24, 0.26, 0.28, 0.30, 0.32, 0.34, 0.36,
    0.38, 0.40, 0.42, 0.44, 0.46, 0.48, 0.50


 };

    std::string url = "https://raw.githubusercontent.com/toml-lang/toml/main/examples/example-v0.5.0.toml";
    std::string outputFile = "data.toml";

    std::cout << "TOML dosyası indiriliyor...\n";
    if (downloadTOML(url, outputFile)) {
        std::cout << "İndirme tamamlandı: " << outputFile << "\n";
    } else {
        std::cout << "İndirme başarısız oldu!\n";
    }

    // Adım 1: Filtreleme
    std::vector<point> all_points = filtrele(&t);

    // Adım 2: Segmentasyon
    std::vector<std::vector<point>> segments = segmentlereAyir(all_points, p.segmentationDistanceThreshHold);
    std::cout << "Toplam bulunan segment sayisi: " << segments.size() << std::endl;
    //p.lineFitThreshHold = 8.0 / segments.size();
    std::cout<<p.lineFitThreshHold <<std::endl;
    // Adım 3: Doğruları Bulma
    std::vector<Line> bulunan_tum_dogrular;
    for (const auto& segment : segments) {
        std::vector<Line> dogrular = findLinesWithSplitAndMerge(segment, p);
        bulunan_tum_dogrular.insert(bulunan_tum_dogrular.end(), dogrular.begin(), dogrular.end());
    }
    // Adım 4: Sonuçları Yazdırma
    std::cout << "\n----------------------------------------\n"
              << "Nihai olarak bulunan dogru sayisi: " << bulunan_tum_dogrular.size() << "\n"
              << "----------------------------------------" << std::endl;
    for (size_t i = 0; i < bulunan_tum_dogrular.size(); ++i) {
        std::cout << "Dogru " << i + 1 << ": Baslangic(" << bulunan_tum_dogrular[i].start.x << ", "
                  << bulunan_tum_dogrular[i].start.y << ") -> Bitis(" << bulunan_tum_dogrular[i].end.x << ", "
                  << bulunan_tum_dogrular[i].end.y << ")" << std::endl;
    }

   const int WIDTH = 800;
    const int HEIGHT = 800;
    const float SCALE = 100.0f;
    const float CENTER_X = WIDTH / 2.0f;
    const float CENTER_Y = HEIGHT / 2.0f;

    // Pencere oluştur
    sf::RenderWindow window(sf::VideoMode({WIDTH, HEIGHT}), "SFML Lidar Gorsellestirme");

    // FONT YÜKLEME
    // ÖNEMLİ: SFML'in dahili fontu yoktur. Bu kodun çalışması için projenizin
    // derleme klasöründe (cmake-build-debug) "arial.ttf" adında bir font dosyası olmalıdır.
    sf::Font font;
    if (!font.loadFromFile("C:/Windows/Fonts/arial.ttf")) {
        std::cerr << "Font dosyasi ('arial.ttf') yuklenemedi! Lutfen dosyanin dogru klasorde oldugundan emin olun." << std::endl;
        return -1;
    }

    // --- ANA ÇİZİM DÖNGÜSÜ (Tek seferlik çizim için) ---
    window.clear(sf::Color::White);

    // --- Izgara Çizimi ---
    sf::Color gridColor(220, 220, 220);
    for (int i = 0; i <= WIDTH; i += 50) {
        sf::Vertex line[] = { sf::Vertex(sf::Vector2f(i, 0), gridColor), sf::Vertex(sf::Vector2f(i, HEIGHT), gridColor) };
        window.draw(line, 2, sf::Lines);
    }
    for (int j = 0; j <= HEIGHT; j += 50) {
        sf::Vertex line[] = { sf::Vertex(sf::Vector2f(0, j), gridColor), sf::Vertex(sf::Vector2f(WIDTH, j), gridColor) };
        window.draw(line, 2, sf::Lines);
    }

    // --- Eksenleri Çiz (Kalın çizgi için RectangleShape kullanılıyor) ---
    sf::RectangleShape xAxis(sf::Vector2f(WIDTH, 2));
    xAxis.setPosition(0, CENTER_Y - 1);
    xAxis.setFillColor(sf::Color::Black);
    window.draw(xAxis);

    sf::RectangleShape yAxis(sf::Vector2f(2, HEIGHT));
    yAxis.setPosition(CENTER_X - 1, 0);
    yAxis.setFillColor(sf::Color::Black);
    window.draw(yAxis);

    // --- Robotun konumu (0,0) ---
    sf::CircleShape robotShape(6.f);
    robotShape.setOrigin(6.f, 6.f); // Merkezi tam ortası yap
    robotShape.setPosition(CENTER_X, CENTER_Y);
    robotShape.setFillColor(sf::Color(0, 200, 0));
    window.draw(robotShape);

    // --- Noktalar (kırmızı) ---
    for (const auto& pnt : all_points) {
        float x = CENTER_X + pnt.x * SCALE;
        float y = CENTER_Y - pnt.y * SCALE;
        sf::CircleShape pointShape(2.f);
        pointShape.setOrigin(2.f, 2.f);
        pointShape.setPosition(x, y);
        pointShape.setFillColor(sf::Color::Red);
        window.draw(pointShape);
    }

    // --- Doğrular (mavi) ve Açıları (mor) ---
    for (const auto& line : bulunan_tum_dogrular) {
        float x1 = CENTER_X + line.start.x * SCALE;
        float y1 = CENTER_Y - line.start.y * SCALE;
        float x2 = CENTER_X + line.end.x * SCALE;
        float y2 = CENTER_Y - line.end.y * SCALE;

        // Kalın çizgi için RectangleShape kullan
        sf::Vector2f p1(x1, y1);
        sf::Vector2f p2(x2, y2);
        sf::Vector2f direction = p2 - p1;
        float length = std::hypot(direction.x, direction.y);
        sf::RectangleShape lineShape(sf::Vector2f(length, 2.f));
        lineShape.setOrigin(0.f, 1.f);
        lineShape.setPosition(p1);
        lineShape.setFillColor(sf::Color::Blue);
        lineShape.setRotation(atan2(direction.y, direction.x) * 180.f / M_PI);
        window.draw(lineShape);

        // Açı metnini yazdır
        double angle = calculateLineAngleDeg(line);
        char angle_text[32];
        snprintf(angle_text, sizeof(angle_text), "%.1f deg", angle);

        sf::Text text(angle_text, font, 12);
        text.setFillColor(sf::Color(128, 0, 128));
        text.setPosition((x1 + x2) / 2 + 10, (y1 + y2) / 2 - 10);
        window.draw(text);
    }

    // --- Kesişen Doğruların Analizi (turuncu kesikli çizgi) ---
    sf::Color dashedColor(255, 165, 0);
    if (bulunan_tum_dogrular.size() >= 2) {
        for (size_t i = 0; i < bulunan_tum_dogrular.size() - 1; ++i) {
            const Line& line1 = bulunan_tum_dogrular[i];
            const Line& line2 = bulunan_tum_dogrular[i + 1];
            if (arePointsEqual(line1.end, line2.start)) {
                const point& intersection_point = line1.end;
                sf::Vector2f intersection_v(
                    CENTER_X + intersection_point.x * SCALE,
                    CENTER_Y - intersection_point.y * SCALE
                );
                draw_dashed_line(window, {CENTER_X, CENTER_Y}, intersection_v, dashedColor, 8.f, 4.f);
            }
        }
    }

    // --- Başlık ---
    sf::Text title("Kirmizi: Noktalar | Mavi: Dogrular | Yesil: Robot (0,0)", font, 14);
    title.setFillColor(sf::Color::Black);
    title.setPosition(10, 10);
    window.draw(title);

    // --- Ekranı göster ---
    window.display();

    // Pencereyi kapatana kadar bekle (daha interaktif bir yöntem)
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }
    }

    return 0;
}

