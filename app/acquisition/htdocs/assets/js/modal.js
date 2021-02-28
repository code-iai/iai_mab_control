function modal(title, body, closeString) {
	var modalObject = document.querySelector('#modal');

	if (!modalObject) {
		const template = document.createElement('template');

		template.innerHTML = (`
		<div class="modal fade" id="modal" tabindex="-1">
			<div class="modal-dialog modal-dialog-scrollable modal-dialog-centered">
				<div class="modal-content">
					<div class="modal-header">
						<h5 class="modal-title"></h5>
						<button type="button" class="btn-close" data-bs-dismiss="modal"></button>
					</div>
					<div class="modal-body"></div>
					<div class="modal-footer">
						<button type="button" class="btn btn-secondary" data-bs-dismiss="modal">` + closeString + `</button>
					</div>
				</div>
			</div>
		</div>
		`).trim();

		modalObject = template.content.childNodes[0];
		document.body.appendChild(modalObject);
	}

	modalObject.querySelector('.modal-title').innerHTML = title;
	modalObject.querySelector('.modal-body').innerHTML = body;
	(new bootstrap.Modal(modalObject)).show();
}
