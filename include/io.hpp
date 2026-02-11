#ifndef IO_HPP
#define IO_HPP

#include "types.hpp"
#include <string>
#include <vector>
#include <fstream>

/**
 * Module I/O pour lire les chemins depuis les fichiers
 * Format attendu: (x1,y1)(x2,y2)(x3,y3)...
 * Compatible avec le format du repo Java original
 */
class PathIO {
public:
    /**
     * Parse une ligne de texte contenant un chemin
     * Format: (x1,y1)(x2,y2)...
     * @param line La ligne à parser
     * @return Le chemin extrait
     */
    static Path parse_line(const std::string& line);
    
    /**
     * Lit un fichier contenant un seul chemin
     * @param filename Le nom du fichier
     * @return Le chemin lu
     */
    static Path read_single_path(const std::string& filename);
    
    /**
     * Lit un fichier contenant plusieurs chemins (un par ligne)
     * @param filename Le nom du fichier
     * @return Vecteur de chemins
     */
    static std::vector<Path> read_multiple_paths(const std::string& filename);
    
    /**
     * Écrit un chemin dans un fichier (pour tests)
     * @param path Le chemin à écrire
     * @param filename Le nom du fichier de sortie
     */
    static void write_path(const Path& path, const std::string& filename);
    
    /**
     * Valide qu'un chemin respecte les contraintes
     * @param path Le chemin à valider
     * @return true si valide, false sinon
     */
    static bool validate_path(const Path& path);
    
    /**
     * Génère un chemin aléatoire (pour tests)
     * @param num_points Nombre de points
     * @param seed Graine pour reproductibilité
     * @return Chemin généré
     */
    static Path generate_random_path(size_t num_points, unsigned seed = 42);
    
    /**
     * Affiche un chemin de manière lisible
     * @param path Le chemin à afficher
     * @param name Nom optionnel pour identifier le chemin
     */
    static void print_path(const Path& path, const std::string& name = "Path");
    
private:
    // Helper pour vérifier les bornes d'un point
    static bool is_point_valid(const IntPoint& p);
};

#endif // IO_HPP