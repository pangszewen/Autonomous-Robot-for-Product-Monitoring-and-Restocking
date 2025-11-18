// Import Firebase SDK
import { initializeApp } from "https://www.gstatic.com/firebasejs/9.23.0/firebase-app.js";
import { getDatabase, ref, onValue } from "https://www.gstatic.com/firebasejs/9.23.0/firebase-database.js";

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

let notifications = JSON.parse(localStorage.getItem('notifications') || '[]');
let previousOutOfStock = new Set();

// Initialize Firebase
try {
  const app = initializeApp(firebaseConfig);
  const database = getDatabase(app);

  // Reference the database path
  const productCountRef = ref(database, "product_counts");
  const stockCountRef = ref(database, "stock_counts");

  // Listen for Realtime Database Changes
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


  onValue(
    stockCountRef, 
    (snapshot) => {
      try {
        const val = snapshot.val();
        if (!val) throw new Error("No stock count data.");

        const outOfStockList = $("#out-of-stock-list");
        outOfStockList.empty();

        Object.entries(val).forEach(([item, data]) => {
          const count = data.count || 0;
          if (count === 0) {
            outOfStockList.append(`<li>${item}</li>`);
          }
        });
      } catch (error) {
        console.error("Error loading stock_counts: ", error);
        displayErrorMessage("Error loading stock data.");
      }
      previousOutOfStock = outOfStockList;
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

function openSidebar() {
  const sidebar = document.getElementById("sidebar");
  const list = document.getElementById("historyList");

  sidebar.classList.add("open");
  list.innerHTML = ""; // clear list

  const history = JSON.parse(localStorage.getItem("notificationHistory")) || [];

  history.forEach(item => {
    const li = document.createElement("li");
    li.textContent = `${item.time}: ${item.message}`;
    list.appendChild(li);
  });
}

/* 
function addNotification(productName) {
  const notification = {
    id: Date.now(),
    product: productName,
    time: new Date().toLocaleString(),
    read: false
  };

  notifications.unshift(notification);
  localStorage.setItem('notifications', JSON.stringify(notifications));
  updateNotificationUI();
  showToast(productName);
}

function updateNotificationUI() {
  const notificationList = document.getElementById('notification-list');
  const badge = document.getElementById('notification-badge');
            
  const unreadCount = notifications.filter(n => !n.read).length;
            
  if (unreadCount > 0) {
    badge.textContent = unreadCount;
    badge.style.display = 'flex';
  } else {
    badge.style.display = 'none';
  }

  if (notifications.length === 0) {
    notificationList.innerHTML = '<div class="empty-state">No notifications yet</div>';
    return;
  }

  notificationList.innerHTML = notifications.map(notif => `
    <div class="notification-item ${notif.read ? '' : 'unread'}">
    <div class="notification-content">
        <div class="notification-title">⚠️ ${notif.product} is out of stock</div>
        <div class="notification-time">${notif.time}</div>
      </div>
    </div>
  `).join('');
}

function showToast(productName) {
  const toast = document.getElementById('toast');
  const toastMessage = document.getElementById('toast-message');
            
  toastMessage.textContent = `${productName} is now out of stock!`;
  toast.classList.add('show');

  setTimeout(() => {
    toast.classList.remove('show');
  }, 5000);
}

window.closeToast = function() {
  document.getElementById('toast').classList.remove('show');
};

window.toggleNotificationPanel = function() {
  const panel = document.getElementById('notification-panel');
  panel.classList.toggle('open');
            
  if (panel.classList.contains('open')) {
    notifications.forEach(n => n.read = true);
    localStorage.setItem('notifications', JSON.stringify(notifications));
    updateNotificationUI();
  }
};

window.clearAllNotifications = function() {
  if (confirm('Are you sure you want to clear all notifications?')) {
    notifications = [];
    localStorage.setItem('notifications', JSON.stringify(notifications));
    updateNotificationUI();
  }
};

// Initialize notification UI
updateNotificationUI();
*/

  
