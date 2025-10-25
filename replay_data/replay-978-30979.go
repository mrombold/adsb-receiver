package main

import (
	"bufio"
	"encoding/json"
	"flag"
	"log"
	"net"
	"os"
	"time"
)

func main() {
	// Command line flags
	file := flag.String("f", "traffic.json", "File containing dump978 JSON data (one JSON object per line)")
	port := flag.String("p", "30979", "Port to listen on")
	delay := flag.Duration("d", 100*time.Millisecond, "Delay between messages")
	loop := flag.Bool("loop", false, "Loop messages continuously")
	
	flag.Parse()
	
	// Read JSON messages from file
	messages, err := readJSONMessages(*file)
	if err != nil {
		log.Fatalf("Failed to read messages: %v", err)
	}
	
	log.Printf("Loaded %d JSON messages from %s", len(messages), *file)
	
	// Start TCP server
	listener, err := net.Listen("tcp", ":"+*port)
	if err != nil {
		log.Fatalf("Failed to start server: %v", err)
	}
	defer listener.Close()
	
	log.Printf("Listening on port %s (dump978 JSON traffic)", *port)
	log.Printf("Delay: %v, Loop: %v", *delay, *loop)
	
	// Accept connections and send messages
	for {
		conn, err := listener.Accept()
		if err != nil {
			log.Printf("Accept error: %v", err)
			continue
		}
		
		log.Printf("Client connected from %s", conn.RemoteAddr())
		go handleClient(conn, messages, *delay, *loop)
	}
}

func readJSONMessages(filename string) ([]string, error) {
	file, err := os.Open(filename)
	if err != nil {
		return nil, err
	}
	defer file.Close()
	
	var messages []string
	scanner := bufio.NewScanner(file)
	lineNum := 0
	
	for scanner.Scan() {
		lineNum++
		line := scanner.Text()
		
		// Skip empty lines and comments
		if len(line) == 0 || line[0] == '#' {
			continue
		}
		
		// Validate it's valid JSON by trying to unmarshal
		var jsonCheck map[string]interface{}
		if err := json.Unmarshal([]byte(line), &jsonCheck); err != nil {
			log.Printf("Warning: Line %d is not valid JSON, skipping: %v", lineNum, err)
			continue
		}
		
		// Check if it has the expected dump978 fields
		if _, hasAddress := jsonCheck["address"]; !hasAddress {
			log.Printf("Warning: Line %d missing 'address' field, skipping", lineNum)
			continue
		}
		
		messages = append(messages, line)
	}
	
	if err := scanner.Err(); err != nil {
		return nil, err
	}
	
	return messages, nil
}

func handleClient(conn net.Conn, messages []string, delay time.Duration, loop bool) {
	defer conn.Close()
	
	writer := bufio.NewWriter(conn)
	sent := 0
	
	for {
		for i, message := range messages {
			// Send JSON message with newline (dump978 format)
			_, err := writer.WriteString(message + "\n")
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
			
			// Log progress every 100 messages
			if sent%100 == 0 {
				log.Printf("Sent %d messages (current: %d/%d)", sent, i+1, len(messages))
			}
			
			// Delay between messages
			time.Sleep(delay)
		}
		
		// Stop if not looping
		if !loop {
			log.Printf("Finished sending %d messages", sent)
			return
		}
		
		// Log when restarting loop
		log.Printf("Restarting loop (sent %d messages so far)", sent)
	}
}