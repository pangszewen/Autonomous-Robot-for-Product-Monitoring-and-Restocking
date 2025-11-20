// Import Firebase SDK
import { initializeApp } from "https://www.gstatic.com/firebasejs/9.23.0/firebase-app.js";
import { getDatabase, ref, onValue, get, push, set} from "https://www.gstatic.com/firebasejs/9.23.0/firebase-database.js";

// Your web app's Firebase configuration
const firebaseConfig = {
  apiKey: "AIzaSyC4o-f1eJHyGBsRv8t2QmbixiktF076Kpo",
  authDomain: "product-monitoring-fe713.firebaseapp.com",
  databaseURL: "https://product-monitoring-fe713-default-rtdb.asia-southeast1.firebasedatabase.app",
  projectId: "product-monitoring-fe713",
  storageBucket: "product-monitoring-fe713.firebasestorage.app",
  messagingSenderId: "778054405641",
  appId: "1:778054405641:web:f191cef429853b3b40b2d8",
  measurementId: "G-RE74HTTLKR"
};
const app = initializeApp(firebaseConfig);
const database = getDatabase(app);

// Initialize Firebase
try {
  // Reference the database path
  const productCountRef = ref(database, "product_counts");
  const stockCountRef = ref(database, "stock_counts");
  let previousOOS = [];

  get(stockCountRef)
  .then(snapshot => {
      if (snapshot.exists()) {
          const data = snapshot.val();

          // Extract out-of-stock items as strings
          Object.entries(data).forEach(([item, obj]) => {
            const count = obj.count || 0;
            if (count === 0) {
              previousOOS.push(item); // store just item names, not <li>
            }
          });

          onValue(
            stockCountRef, 
            (snapshot) => {
              try {
                const val = snapshot.val();
                if (!val) throw new Error("No stock count data.");

                const outOfStockList = $("#out-of-stock-list");
                outOfStockList.empty();

                const currentOutOfStock = [];

                Object.entries(val).forEach(([item, data]) => {
                  const count = data.count || 0;
                  if (count === 0) {
                    outOfStockList.append(`<li>${item}</li>`);
                    currentOutOfStock.push(item);
                    
                    // Check if this item is NEW (wasn't in previous set)
                    if (!previousOOS.includes(item)) {
                      const message = `${item} is OUT OF STOCK!`;
                      showToast(message);
                      playSound();
                      saveNotification(message);
                    }
                  }
                });

              } catch (error) {
                console.error("Error loading stock_counts: ", error);
                displayErrorMessage("Error loading stock data.", error);
              }
              previousOOS = currentOutOfStock;
          });
      } else {
          console.log("No data found in stock_counts.");
      }
  })
  .catch(error => {
      console.error("Error initializing previousOOS:", error);
  });

  onValue(
    productCountRef, 
    (snapshot) => {
      try {
        const val = snapshot.val();
        if (!val) {
          throw new Error("No data found in Firebase Realtime Database.");
        }
        console.log("Received snapshot:", val);

        Object.entries(val).forEach(([item, data]) => {
          item = item.replaceAll(" ", "_");
          const count = data.count || 0;
          const selector = `#${item}-count`;

          $(selector).text(count);

          if (count < 3) {
            $(selector).css("color", "red");
          } else {
            $(selector).css("color", "black");
          }
        });
    }catch (error) {
      console.error("Error loading product_counts: ", error);
      displayErrorMessage("Error loading product inventory.");
    }
  });

  // Error Message Display Function
  function displayErrorMessage(message) {
    const errorBox = $("#error-box");
    if (errorBox.length === 0) {
      // Create error message box if not exists
      $("body").prepend(`
        <div id="error-box" style="color: red; background: lightpink; padding: 10px; text-align: center; display: none;">
          ${message}
        </div>
      `);
    } else {
      $("#error-box").text(message);
    }
    $("#error-box").fadeIn("slow").delay(5000).fadeOut("slow");
  }
} catch (error) {
  // Handle Firebase initialization errors
  console.error("Error initializing Firebase: ", error);
  alert("Failed to initialize Firebase. Please check the configuration.");
}

function saveNotification(message) {
    const notifRef = ref(database, "notifications");
    // Create a new child with an auto-generated ID
    const newNotifRef = push(notifRef);
    const id = newNotifRef.key;

    // Store the notification
    set(newNotifRef, {
        id: id,
        message: message,
        created_at: Date.now(),
        read: false
    });

    console.log("Saved Notification with ID:", id);
}

// Open/close sidebar & load history
function toggleNotificationSidebar() {
    const sidebar = document.getElementById("notification-sidebar");
    const list = document.getElementById("notification-history");

    // Toggle 'open' class
    const isOpen = sidebar.classList.contains("open");
    if (isOpen) {
        sidebar.classList.remove("open"); // close
        return; // no need to reload notifications
    }

    sidebar.classList.add("open"); // open
    list.innerHTML = ""; // clear current UI

    // Load notifications from Firebase
    const notifRef = ref(database, "notifications");
    onValue(notifRef, (snapshot) => {
        list.innerHTML = ""; // clear UI

        const data = snapshot.val();
        if (!data) {
            list.innerHTML = "<li>No notifications yet.</li>";
            return;
        }

        // Convert Firebase object to array and sort newest first
        const arr = Object.values(data).sort((a, b) => b.created_at - a.created_at);

        arr.forEach(n => {
            const li = document.createElement("li");
            const date = new Date(n.created_at).toLocaleString();
            li.textContent = `${date} — ${n.message}`;
            list.appendChild(li);
        });
    });
}

// Toast notification
function showToast(msg) {
  const toast = document.getElementById("toast");
  toast.textContent = msg;
  toast.className = "show";

  setTimeout(() => toast.className = toast.className.replace("show", ""), 3000);
}

// Play sound
function playSound() {
  document.getElementById("notif-sound").play();
}

// Button click opens sidebar
document.getElementById("notification-btn").addEventListener("click", () => {
  toggleNotificationSidebar();
});

  
