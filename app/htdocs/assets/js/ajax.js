function post_json(url, data, success, error, always) {
	const xhr = new XMLHttpRequest();
	xhr.open('POST', url, true);
	xhr.setRequestHeader('Content-Type', 'application/json');

	xhr.onreadystatechange = function() {
		if (this.readyState == XMLHttpRequest.DONE) {
			if (this.status == 200) {
				success(this.responseText);
			} else {
				error(this.responseText);
			}

			if (typeof always != 'undefined') {
				always(this.responseText);
			}
		}
	};

	if (data) {
		xhr.send(JSON.stringify(data));
	} else {
		xhr.send();
	}
}
