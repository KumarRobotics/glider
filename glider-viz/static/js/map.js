// Wait for the DOM to be fully loaded before initializing the map
document.addEventListener('DOMContentLoaded', function() {
    // Initialize map with center coordinates and appropriate zoom level
    var map = L.map('map').setView([39.941326, -75.199492], 16);

    // Try different tile providers - one of these should work
    // 1. Mapbox Satellite
    var mapboxDetailed = L.tileLayer('https://api.mapbox.com/styles/v1/{id}/tiles/{z}/{x}/{y}?access_token={accessToken}', {
        attribution: 'Â© <a href="https://www.mapbox.com/about/maps/">Mapbox</a>',
        maxZoom: 30,
        id: 'mapbox/satellite-v9',
        accessToken: 'pk.eyJ1IjoiamFzb25haCIsImEiOiJjbThxOWt5ZnMwa3NxMmtwdTEwYjJ1ajZ3In0.LGKzoIXaNi9j7lXhypfBPQ'
    }).addTo(map);
    
    // 2. OpenStreetMap as fallback
    var osm = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        maxZoom: 19,
        attribution: 'Â© OpenStreetMap contributors'
    });

    var baseLayers = {
        "MapBox Satellite": mapboxDetailed,
        "OpenStreetMap": osm
    };

    L.control.layers(baseLayers).addTo(map);

    // Add a console log to check if the map initialized
    console.log("Map initialized with center:", map.getCenter(), "zoom:", map.getZoom());

    // Create layers for your data
    var pointsLayer = L.layerGroup().addTo(map);
    var pathLayer = L.layerGroup().addTo(map);
    var objectsLayer = L.layerGroup().addTo(map);
    var connectionsLayer = L.layerGroup().addTo(map);

    // Create a simple arrow icon that can be rotated
    // Create a simple arrowhead icon that can be rotated
    function createArrowIcon(rotation) {
        console.log("Creating arrow icon with rotation:", rotation);
        
        var svgArrow = `
            <svg width="20" height="20" viewBox="0 0 20 20" xmlns="http://www.w3.org/2000/svg" style="transform: rotate(${rotation}deg);">
                <defs>
                    <filter id="shadow" x="-50%" y="-50%" width="200%" height="200%">
                        <feDropShadow dx="1" dy="1" stdDeviation="1" flood-color="#000" flood-opacity="0.5"/>
                    </filter>
                </defs>
                <!-- Triangular arrowhead pointing up (north) -->
                <polygon points="10,3 16,15 4,15" fill="#FF3333" stroke="#FFFFFF" stroke-width="1.5" filter="url(#shadow)"/>
            </svg>
        `;
        
        return L.divIcon({
            html: svgArrow,
            iconSize: [20, 20],
            iconAnchor: [10, 10],
            popupAnchor: [0, -10],
            className: 'custom-arrow-icon'
        });
    }

    // Create a legend
    var legend = L.control({position: 'bottomright'});

    legend.onAdd = function (map) {
        var div = L.DomUtil.create('div', 'info legend');
        div.style.backgroundColor = 'rgba(255, 255, 255, 0.8)';
        div.style.padding = '10px';
        div.style.borderRadius = '5px';
        div.style.boxShadow = '0 0 15px rgba(0,0,0,0.2)';
        
        // Add title
        div.innerHTML = '<h4 style="margin-top: 0; margin-bottom: 10px; text-align: center;">Legend</h4>';
        
        // Define your legend items here
        // Format: [color, label]
        var items = [
            ['#FF3333', 'Position'],
        ];
        
        // Collect unique colors from objects to populate legend dynamically
        var objectColors = {};
        
        // Function to update legend with object colors
        window.updateLegendWithObjectColors = function(objects) {
            if (!objects || objects.length === 0) return;
            
            objects.forEach(function(obj) {
                if (obj.color && obj.label) {
                    objectColors[obj.color] = obj.label;
                }
            });
            
            // Rebuild items array
            items = [['#FF3333', 'Position']];
            
            // Add object colors to items
            for (var color in objectColors) {
                items.push([color, objectColors[color]]);
            }
            
            // Update legend HTML
            updateLegendHTML();
        };
        
        // Function to update legend HTML
        function updateLegendHTML() {
            var legendHTML = '<h4 style="margin-top: 0; margin-bottom: 10px; text-align: center;">Legend</h4>';
            
            // Add each legend item
            for (var i = 0; i < items.length; i++) {
                if (i === 0) {
                    // Special case for position - show arrowhead icon instead of circle
                    legendHTML += 
                        '<div style="display: flex; align-items: center; margin-bottom: 5px;">' +
                            '<span style="width: 16px; height: 16px; display: inline-block; margin-right: 5px;">' +
                                '<svg width="16" height="16" viewBox="0 0 16 16" xmlns="http://www.w3.org/2000/svg">' +
                                    '<polygon points="8,2 12,12 4,12" fill="#FF3333" stroke="#FFFFFF" stroke-width="1"/>' +
                                '</svg>' +
                            '</span>' +
                            '<span>' + items[i][1] + '</span>' +
                        '</div>';
                } else {
                    legendHTML += 
                        '<div style="display: flex; align-items: center; margin-bottom: 5px;">' +
                            '<span style="background:' + items[i][0] + '; width: 15px; height: 15px; border-radius: 50%; display: inline-block; margin-right: 5px; border: 1px solid #FFF;"></span> ' +
                            '<span>' + items[i][1] + '</span>' +
                        '</div>';
                }
            }
            
            div.innerHTML = legendHTML;
        }
        
        // Initial legend HTML update
        updateLegendHTML();
        
        return div;
    };

    // Add legend to map
    legend.addTo(map);

    // Connect to WebSocket
    var socket = io();

    // Listen for GPS updates
    socket.on('gps_update', function(data) {
        console.log("Received GPS update:", data);
        
        // Clear previous markers and paths
        pointsLayer.clearLayers();
        pathLayer.clearLayers();
        
        if (data.points.length === 0) return;
        
        // Set the color to use for GPS track
        var trackColor = '#FF3333';  // Red color
        
        // Extract all coordinates for the path
        var pathCoords = data.points.map(point => [point.lat, point.lon]);
        
        // Draw the connecting line for all points
        if (pathCoords.length > 1) {
            L.polyline(pathCoords, {
                color: trackColor,
                weight: 3,
                opacity: 0.7
            }).addTo(pathLayer);
        }
        
        // Add only the most recent point with arrow icon showing orientation
        var lastPoint = data.points[data.points.length - 1];
        
        // Get the rotation angle (yaw) from the data, default to 0 if not available
        var rotation = lastPoint.yaw || 0;
        
        console.log("Creating arrow at position:", lastPoint.lat, lastPoint.lon, "with rotation:", rotation);
        
        // Create arrow marker with orientation
        var arrowIcon = createArrowIcon(rotation);
        
        var marker = L.marker([lastPoint.lat, lastPoint.lon], {
            icon: arrowIcon
        })
        .bindPopup((lastPoint.popup || "Position") + "<br>Heading: " + rotation.toFixed(1) + "°")
        .addTo(pointsLayer);
        
        console.log("Arrow marker created and added to map:", marker);
    });

    // Force a map refresh
    setTimeout(function() {
        map.invalidateSize();
    }, 100);
});
