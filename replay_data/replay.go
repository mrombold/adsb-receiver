package main

import (
	"bufio"
	"flag"
	"log"
	"net"
	"os"
	"time"
)

func main() {
	// Command line flags
	file := flag.String("f", "frames.txt", "File containing dump978 frames (one per line)")
	port := flag.String("p", "30978", "Port to listen on")
	delay := flag.Duration("d", 100*time.Millisecond, "Delay between frames")
	loop := flag.Bool("loop", false, "Loop frames continuously")
	
	flag.Parse()

	// Read frames from file
	frames, err := readFrames(*file)
	if err != nil {
		log.Fatalf("Failed to read frames: %v", err)
	}

	log.Printf("Loaded %d frames from %s", len(frames), *file)

	// Start TCP server
	listener, err := net.Listen("tcp", ":"+*port)
	if err != nil {
		log.Fatalf("Failed to start server: %v", err)
	}
	defer listener.Close()

	log.Printf("Listening on port %s", *port)
	log.Printf("Delay: %v, Loop: %v", *delay, *loop)

	// Accept connections and send frames
	for {
		conn, err := listener.Accept()
		if err != nil {
			log.Printf("Accept error: %v", err)
			continue
		}

		log.Printf("Client connected from %s", conn.RemoteAddr())
		go handleClient(conn, frames, *delay, *loop)
	}
}

func readFrames(filename string) ([]string, error) {
	file, err := os.Open(filename)
	if err != nil {
		return nil, err
	}
	defer file.Close()

	var frames []string
	scanner := bufio.NewScanner(file)

	for scanner.Scan() {
		line := scanner.Text()
		
		// Skip empty lines and comments
		if len(line) == 0 || line[0] == '#' || line[0] == '!' {
			continue
		}

		// Only keep lines that start with + (uplink frames)
		if line[0] == '+' {
			frames = append(frames, line)
		}
	}

	if err := scanner.Err(); err != nil {
		return nil, err
	}

	return frames, nil
}

func handleClient(conn net.Conn, frames []string, delay time.Duration, loop bool) {
	defer conn.Close()
	
	writer := bufio.NewWriter(conn)
	sent := 0

	for {
		for i, frame := range frames {
			// Send frame with newline
			_, err := writer.WriteString(frame + "\n")
			if err != nil {
				log.Printf("Write error: %v", err)
				return
			}

			// Flush to ensure it's sent
			err = writer.Flush()
			if err != nil {
				log.Printf("Flush error: %v", err)
				return
			}

			sent++

			// Log progress every 100 frames
			if sent%100 == 0 {
				log.Printf("Sent %d frames (current: %d/%d)", sent, i+1, len(frames))
			}

			// Delay between frames
			time.Sleep(delay)
		}

		// Stop if not looping
		if !loop {
			log.Printf("Finished sending %d frames", sent)
			return
		}
	}
}