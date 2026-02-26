console.log("Bandwidth JS Loaded");

document.getElementById("runTest").addEventListener("click", runBandwidthTest);

async function runBandwidthTest() {
    console.log("Running bandwidth test...");

    const status = document.getElementById("status");
    const downloadEl = document.getElementById("download");
    const uploadEl = document.getElementById("upload");
    const latencyEl = document.getElementById("latency");
    const jitterEl = document.getElementById("jitter");
    const lossEl = document.getElementById("loss");

    status.textContent = "Running test...";
    status.style.color = "yellow";

    try {
        const response = await fetch("/run_iperf");
        const data = await response.json();

        if (data.error) throw new Error(data.error);

        downloadEl.textContent = data.download;
        uploadEl.textContent = data.upload;
        latencyEl.textContent = data.latency;
        jitterEl.textContent = data.jitter;
        lossEl.textContent = data.loss;

        status.textContent = "Test complete!";
        status.style.color = "limegreen";

    } catch (err) {
        console.error("Error running test:", err);
        status.textContent = "⚠️ " + err.message;
        status.style.color = "red";
    }
}

