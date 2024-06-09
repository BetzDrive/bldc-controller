// build.rs
extern crate bindgen;

fn main() {
    let bindings = bindgen::Builder::default()
        .header("common/src/constants.cpp")
        .generate()
        .expect("Unable to generate bindings");

    bindings
        .write_to_file("src/bindings.rs")
        .expect("Couldn't write bindings!");
}

