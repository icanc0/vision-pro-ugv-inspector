package main

import (
	"github.com/go-chi/chi/v5"
	"github.com/go-chi/chi/v5/middleware"
	"log"
	"net/http"
	"os"
	"os/user"
	"path/filepath"
)

const (

	//filename = "C:\\Users\\icanc\\Documents\\PanoStitchOutput.jpg"
	port = ":8080"
)

var filename string

func main() {

	currentUser, err := user.Current()
	if err != nil {
		log.Fatalf("Error getting current user: %v", err)
	}

	// Construct the file path
	filename = filepath.Join(currentUser.HomeDir, "Documents", "PanoStitchOutput.jpg")

	r := chi.NewRouter()

	r.Use(middleware.Logger)
	r.Use(middleware.Recoverer)

	r.Get("/", serveFile)

	log.Printf("Server starting on port %s", port)
	log.Fatal(http.ListenAndServe(port, r))
}

func serveFile(w http.ResponseWriter, r *http.Request) {
	file, err := os.Open(filename)
	if err != nil {
		http.Error(w, "File not found", http.StatusNotFound)
		return
	}
	defer file.Close()

	//// HASHING WOOOHOOO
	//hash := sha256.New()
	//if _, err := io.Copy(hash, file); err != nil {
	//	http.Error(w, "Error calculating hash", http.StatusInternalServerError)
	//	return
	//}
	//
	//fmt.Printf("File hash: %x\n", hash.Sum(nil))

	file.Seek(0, 0)

	http.ServeFile(w, r, filename)
}
